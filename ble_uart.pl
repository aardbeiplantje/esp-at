#!/usr/bin/perl

BEGIN {
    $ENV{LC_ALL} = "C";
};

use strict; use warnings;

no warnings 'once';

use FindBin;
use Getopt::Long;
use Errno qw(EAGAIN EINTR);
use POSIX ();
use List::Util ();

BEGIN {
    my $timezone = POSIX::tzname();
    $ENV{TZ} = readlink('/etc/localtime') =~ s|^.*/zoneinfo/||gr;
    POSIX::tzset();
    $::DOLLAR_ZERO = $0;
    $0 = "ble:uart";
};

$::APP_NAME = "ble:uart";

my $cfg = handle_cmdline_options();
my ($rin, $win, $ein, $connections) = ("", "", "", {});
eval {
    main_loop($cfg->{targets});
};
if($@){
    chomp(my $err = $@);
    if($err =~ m/^(TERM|INT) signal, exiting$/){
        logger::info("exiting cleanly: $err");
        exit 0;
    } else {
        logger::error("main loop error: $err");
        exit 1;
    }
}

exit;

sub main_loop {
    my ($tgts) = @_;
    return unless @{$tgts//[]};
    logger::debug("starting main loop with targets", $tgts);

    # for possible  socket problems, although we do non blocking very fast
    local $SIG{PIPE} = "IGNORE";
    # makes syscalls restarted
    local $SIG{HUP}  = "IGNORE";

    # our clean exiting
    $::DATA_LOOP = 1;
    my $exit_handler_sub = sub {
        $::DATA_LOOP = 0; die "$_[0] signal, exiting\n"
    };
    local $SIG{INT}  = $exit_handler_sub;
    local $SIG{TERM} = $exit_handler_sub;

    # we start non-sleepy
    my $s_timeout = 0;

    # initialize our targets
    connect_tgt($connections, $_) for @{$tgts};

    # our input reader
    my $reader = input::tty->new();
    my $response_buffer = "";

    # main loop
    eval {
    while($::DATA_LOOP){

        # check all connections, and if they have an empty outbox, exit
        if($::DATA_LOOP_EXIT_WANTED){
            my $all_empty = 1;
            foreach my $c (values %{$connections}){
                logger::debug("checking connection $c->{_log_info} (fd: $c->{_fd}), buffer: ".length($c->{_outboxbuffer}//"")." bytes");
                if(length($c->{_outboxbuffer}//"") > 0){
                    $all_empty = 0;
                    last;
                }
            }
            if($all_empty){
                logger::info("all connections have empty outbox, exiting");
                $::DATA_LOOP = 0;
                last;
            }
        }

        # reset select() vecs
        $rin = "";
        $win = "";
        $ein = "";

        # input
        vec($rin, $reader->infd(), 1) = 1 if $reader and $reader->infd();

        # select() vec handling
        my @shuffled_conns = List::Util::shuffle(sort keys %{$connections});
        foreach my $fd (@shuffled_conns){
            logger::debug("checking connection $fd");
            my $c = $connections->{$fd};
            vec($rin, $fd, 1) = 1;

            # other stuff to write?
            vec($win, $fd, 1) = $c->need_write();

            # need select() timeout
            my $tm = $c->need_timeout();
            $s_timeout = $tm if defined $tm and !defined $s_timeout and $tm <= ($s_timeout//86400);
        }
        $s_timeout = 0 if defined $s_timeout and $s_timeout < 0;

        # our main loop
        logger::info("will TIMEOUT: ".($s_timeout//"not"))
            if ($::LOGLEVEL//0) >= 7;
        $ein |= $rin | $win;
        my $r = select(my $rout = $rin, my $wout = $win, my $eout = $ein, $s_timeout // 1);
        if($r == -1){
            $!{EINTR} or $!{EAGAIN} or logger::error("select problem: $!");
            next;
        }

        # check IN|OUT
        if($reader and vec($rout, $reader->infd(), 1)){
            my $data = $reader->do_read();
            if(defined $data){
                logger::debug(">>TTY>>".length($data)." bytes read from TTY");
                # send data to first-and-only connection
                foreach my $c (values %{$connections}){
                    $c->{_outboxbuffer} .= $data;
                    last;
                }
            }
        }

        # anything to read from the remote connections?
        foreach my $fd (@shuffled_conns){
            my $c = $connections->{$fd};
            eval {
                # check error
                if(vec($eout, $fd, 1)){
                    die "Error select: bad FD $fd\n";
                }

                # handle read if select says so
                if(vec($rout, $fd, 1)){
                    my $read_ok = $c->do_read(\$response_buffer);
                    if(!$read_ok){
                        # EOF
                        removing_tgt($connections, $c);
                        return;
                    }
                }

                # handle write if select says so
                $c->do_write() if vec($wout, $fd, 1);
            };
            if($@){
                chomp(my $err = $@);
                do {$::DATA_LOOP = 0; last} if $err =~ m/^QUIT at .* line \d+/;
                logger::error($err);
                removing_tgt($connections, $c);
            }
        }

        # make new connections
        if(keys %{$connections} != @{$tgts}){
            foreach my $t (@{$tgts}){
                next if grep {$t->{b} eq $_->{b}} values %{$connections};
                eval {connect_tgt($connections, $t)};
                if($@){
                    chomp(my $err = $@);
                    do {$::DATA_LOOP = 0; last} if $err =~ m/^QUIT at .* line \d+/;
                    logger::error("problem reconnecting [$t->{b},key:$t->{k}]: $err");
                }
            }
        }

        # if we have a response buffer, write it to the TTY
        if(length($response_buffer) > 0){
            logger::debug(">>TTY>>".length($response_buffer)." bytes to write to TTY");
            my $color_ok = $reader->{_color_ok} // 1;
            my $prefix = ($reader->{_utf8_ok}  // 1) ? "↳ " : "> ";
            $prefix = $colors::green_color.$prefix.$colors::reset_color if $color_ok;
            my $c_reset = $colors::reset_color;
            $c_reset = "" unless $color_ok;
            my $c_resp = $response_buffer =~ m/^\+ERROR:/ ? $colors::red_color : $colors::yellow_color1;
            $c_resp = "" unless $color_ok;
            $reader->{_rl}->save_prompt();
            $reader->{_rl}->clear_message();
            foreach my $l (split /\r?\n/, $response_buffer){
                $reader->{_rl}->message($prefix.$c_resp.$l.$c_reset);
            }
            $reader->{_rl}->crlf();
            $reader->{_rl}->restore_prompt();
            $reader->{_rl}->on_new_line();
            $reader->{_rl}->redisplay();
            substr($response_buffer, 0, length($response_buffer), '');
        }

        # next select timeout?
        $s_timeout = (keys %{$connections} != @{$tgts})?1:undef;
    }
    };
    chomp(my $err = $@);

    # handle clean exits
    removing_tgt($connections, $_) for values %{$connections};

    # cleanup reader
    $reader->cleanup() if $reader;

    # relay error if any
    die "$err\n" if $err and $err !~ m/^(TERM|INT) signal, exiting$/;
    return
}

sub removing_tgt {
    my ($conns, $c) = @_;
    return unless defined $c and defined $c->{_fd};
    logger::info("cleanup $c->{_log_info} (fd: $c->{_fd})");
    vec($rin, $c->{_fd}, 1) = 0;
    vec($win, $c->{_fd}, 1) = 0;
    delete $conns->{$c->{_fd}};
    $c->cleanup();
    return;
}

sub connect_tgt {
    my ($conns, $c) = @_;
    my $n = ble::uart->new({%$c});
    my $fd = $n->init();
    return unless defined $fd;
    $conns->{$fd} = $n;
    return;
}

sub handle_cmdline_options {
    GetOptions(
        my $opts = {
            # defaults for the options go here
        },
        "loglevel=i",
        "manpage|man|m!",
        "help|h|?!",
    ) or utils::usage(-exitval => 1);
    utils::usage(-verbose => 1, -exitval => 0) if $opts->{help};
    utils::usage(-verbose => 2, -exitval => 1) if $opts->{manpage};
    $opts->{loglevel}  = $::APP_ENV{LOGLEVEL} // $opts->{loglevel};

    my @targets;
    foreach my $tgt (@ARGV){
        my @r = split m/=|,/, $tgt, 3;
        my $tgt = {
            k => $r[0],
            b => $r[1],
            l => {split m/=|,/, $r[2]//""}
        };
        if(!$tgt->{k} and !$tgt->{b}){
            logger::error("need key for target");
            utils::usage(-exitval => 1);
        }
        if(!$tgt->{b}){
            logger::error("need bluetooth address for target '$tgt->{k}'");
            utils::usage(-exitval => 1);
        }
        push @targets, $tgt;
    }
    $opts->{targets} = \@targets;
    return $opts;
}

package input::tty;
use strict; use warnings;

our $BASE_DIR;
our $HISTORY_FILE;
our @cmds;

BEGIN {
    $BASE_DIR     //= $ENV{BLE_UART_DIR} // (glob('~/.ble_uart'))[0];
    $HISTORY_FILE //= $ENV{BLE_UART_HISTORY_FILE} // "${BASE_DIR}_history";
    @cmds           = qw(/exit /quit /history /help /debug /nodebug /logging /nologging);
};

BEGIN {
package colors;

our $red_color     = "\033[0;31m";
our $green_color   = "\033[0;32m";
our $yellow_color1 = "\033[0;33m";
our $blue_color1   = "\033[0;34m";
our $blue_color2   = "\033[38;5;25;1m";
our $blue_color3   = "\033[38;5;13;1m";
our $magenta_color = "\033[0;35m";
our $cyan_color    = "\033[0;36m";
our $white_color   = "\033[0;37m";
our $reset_color   = "\033[0m";
};

sub new {
    my ($class, $cfg) = @_;
    $cfg //= {};
    my $self = bless {%$cfg}, ref($class)||$class;
    $self->{_ttyoutbuffer} = "";
    $self->setup_readline();
    return $self;
}

sub infd {
    my ($self) = @_;
    return fileno($self->{_rl}->IN());
}

sub infh {
    my ($self) = @_;
    return $self->{_rl}->IN();
}

sub outfd {
    my ($self) = @_;
    return fileno($self->{_rl}->OUT());
}

sub outfh {
    my ($self) = @_;
    return $self->{_rl}->OUT();
}

sub do_read {
    my ($self) = @_;
    logger::debug(">>TTY>> waiting for input from TTY");
    $self->{_rl}->callback_read_char();
    if(length($self->{_ttyoutbuffer}//"")){
        logger::debug(">>TTY>> returning output buffer with ".length($self->{_ttyoutbuffer})." bytes");
        my $out = $self->{_ttyoutbuffer};
        $self->{_ttyoutbuffer} = "";
        return $out;
    }
    return;
}

sub chat_word_completions_cli {
    my ($text, $line, $start, $end) = @_;
    $line =~ s/ +$//g;
    my @rcs = ();
    my @wrd = split m/\s+/, $line, -1;
    logger::debug("W: >".join(", ", @wrd)."<\n");
    foreach my $w (@wrd) {
        next unless $w =~ m|^/|;
        foreach my $k (@cmds) {
            push @rcs, $k if !index($k, $w) or $k eq $w;
        }
    }
    logger::debug("R: >".join(", ", @rcs)."<");
    return '', @rcs;
}

sub setup_readline {
    my ($self) = @_;
    local $ENV{PERL_RL} = 'Gnu';
    local $ENV{TERM}    = $ENV{TERM} // 'vt220';
    eval {require Term::ReadLine; require Term::ReadLine::Gnu};
    if($@){
        logger::error("Please install Term::ReadLine and Term::ReadLine::Gnu\n\nE.g.:\n  sudo apt install libterm-readline-gnu-perl");
        exit 1;
    }
    my $term = Term::ReadLine->new("aicli");
    $term->read_init_file("$BASE_DIR/inputrc");
    $term->ReadLine('Term::ReadLine::Gnu') eq 'Term::ReadLine::Gnu'
        or die "Term::ReadLine::Gnu is required\n";

    # color support?
    $self->{_color_ok} = ($ENV{COLORTERM} && $ENV{COLORTERM} =~ /color/i);

    # UTF-8 support?
    my $utf8_ok = 1;
    eval {$term->enableUTF8()};
    $utf8_ok = 0 if $@;
    $self->{_utf8_ok} = $utf8_ok;

    $term->using_history();
    $term->ReadHistory($HISTORY_FILE);
    $term->clear_signals();
    my $attribs = $term->Attribs();
    $attribs->{attempted_completion_function} = \&chat_word_completions_cli;
    $attribs->{ignore_completion_duplicates}  = 1;
    my ($t_ps1, $t_ps2) = create_prompt($self);
    $term->callback_handler_install(
        $t_ps1,
        sub {
            $self->rl_cb_handler($t_ps1, $t_ps2, @_);
            return;
        }
    );
    $self->{_rl} = $term;
    return;
}

sub cleanup {
    my ($self) = @_;
    $self->{_rl}->callback_handler_remove();
    $self->{_rl}->WriteHistory($HISTORY_FILE);
    return;
}

sub rl_cb_handler {
    my ($self, $t_ps1, $t_ps2, $line) = @_;
    if(!defined $line){
        $self->{_rl}->callback_handler_remove();
        $::DATA_LOOP = 0; # exit the main loop
        return;
    }
    $self->{_rl}->set_prompt($t_ps1);
    my $buf = \($self->{_buf} //= '');
    if($line !~ m/^$/ms){
        logger::debug(">>TTY>>".length($line)." bytes read from TTY: $line");
        # we have data, are we at init (empty buffer)?
        if(!length($$buf)){
            my $r_val = handle_command($line);
            if(defined $r_val){
                # handle_command handled it, so we can log/continue
                $$buf = '';
                $line =~ s/^\s+//;
                $line =~ s/\s+$//;
                $line =~ s/\r?\n$//;
                $self->{_rl}->addhistory($line);
                $self->{_rl}->WriteHistory($HISTORY_FILE);
                $self->{_rl}->set_prompt($t_ps1);
                if($r_val){
                    $::DATA_LOOP = 0; # exit the main loop
                }
                return;
            }
        }
        # handle_command did not handle it OR we already had a buffer,
        if(0){
            # add to buffer until we have an empty line entered
            $$buf .= "$line\n";
            $self->{_rl}->set_prompt($t_ps2);
        } else {
            # just process the line
            $line =~ s/^\s+//;
            $line =~ s/\s+$//;
            $line =~ s/\r?\n$//;
            $$buf .= "$line\n";
            $self->{_rl}->addhistory($line);
            $self->{_rl}->WriteHistory($HISTORY_FILE);
            $self->{_rl}->set_prompt($t_ps1);
            $self->{_rl}->on_new_line_with_prompt();
            $self->{_ttyoutbuffer} .= $$buf;
            $$buf = '';
        }
        return;
    } else {
        logger::debug(">>TTY>> empty line read from TTY, processing buffer");
        # empty line, this is command execution if we have a buffer
        if(length($$buf)){
            logger::debug("BUF: >>$$buf<<");
            $self->{_rl}->addhistory($$buf);
            $self->{_rl}->WriteHistory($HISTORY_FILE);
            $self->{_rl}->set_prompt($t_ps1);
            $self->{_rl}->on_new_line_with_prompt();
            $self->{_ttyoutbuffer} .= $$buf;
            $$buf = '';
        }
        return;
    }
    return;
}

sub create_prompt {
    # https://jafrog.com/2013/11/23/colors-in-terminal.html
    # https://ss64.com/bash/syntax-colors.html
    my ($self) = @_;
    my $PR =
           $ENV{BLE_UART_PROMPT}
        // $ENV{BLE_UART_PROMPT_DEFAULT}
        // 'AT';
    my $color_ok = $self->{_color_ok};
    my $utf8_ok  = $self->{_utf8_ok};

    my $prompt_term1 = $utf8_ok ? "❲$PR❳►" : "$PR>";
    my $prompt_term2 = $utf8_ok ? "│ " : "| ";
    if ($color_ok) {
        $prompt_term1 = $ENV{BLE_UART_PS1} // $colors::blue_color3.$prompt_term1.$colors::reset_color;
        $prompt_term2 = $ENV{BLE_UART_PS2} // $colors::blue_color3.$prompt_term2.$colors::reset_color;
    }
    my $ps1 = eval "return \"$prompt_term1\"" || ($utf8_ok ? '► ' : '> ');
    my $ps2 = eval "return \"$prompt_term2\"" || ($utf8_ok ? '│ ' : '| ');
    return ($ps1, $ps2);
}

sub handle_command {
    my ($line) = @_;
    logger::debug("Command: $line");
    if ($line =~ m|^/exit| or $line =~ m|^/quit|) {
        return 1;
    }
    if ($line =~ m|^/history|) {
        print do {open(my $_hfh, '<', $HISTORY_FILE) or die "Failed to read $HISTORY_FILE: $!\n"; local $/; <$_hfh>};
        return 0;
    }
    if ($line =~ m|^/debug|) {
        $ENV{uc($::APP_NAME?$::APP_NAME."_LOGLEVEL":"LOGLEVEL") =~ s/\W/_/gr} = "DEBUG";
        return 0;
    }
    if ($line =~ m|^/nodebug|) {
        $ENV{uc($::APP_NAME?$::APP_NAME."_LOGLEVEL":"LOGLEVEL") =~ s/\W/_/gr} = "INFO";
        return 0;
    }
    if ($line =~ m|^/logging|) {
        $ENV{uc($::APP_NAME?$::APP_NAME."_LOGLEVEL":"LOGLEVEL") =~ s/\W/_/gr} = "INFO";
        return 0;
    }
    if ($line =~ m|^/nologging|) {
        $ENV{uc($::APP_NAME?$::APP_NAME."_LOGLEVEL":"LOGLEVEL") =~ s/\W/_/gr} = "NONE";
        return 0;
    }
    if ($line =~ m|^/help|) {
        print join(", ", @cmds)."\n";
        return 0;
    }
    if ($line =~ m|^/|) {
        print "Unknown command: $line\n";
        return 0;
    }
    return;
}

package ble::uart;
use strict; use warnings;

use Errno qw(EAGAIN EINTR EINPROGRESS);
use Fcntl qw(F_SETFL O_RDWR O_NONBLOCK);
use Socket;

# constants for BLE UART (Nordic UART Service) UUIDs
use constant NUS_SERVICE_UUID => "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
use constant NUS_RX_CHAR_UUID => "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
use constant NUS_TX_CHAR_UUID => "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

# constants for BLUETOOTH that come from bluez

# Bluetooth address format
use constant AF_BLUETOOTH     => 31;
use constant PF_BLUETOOTH     => 31;

# Bluetooth socket types
use constant BT_SECURITY        => 4;
use constant BT_SECURITY_SDP    => 0;
use constant BT_SECURITY_LOW    => 1;
use constant BT_SECURITY_MEDIUM => 2;
use constant BT_SECURITY_HIGH   => 3;
use constant BT_SECURITY_FIPS   => 4;

# L2CAP constants
use constant BT_SNDMTU   => 12;
use constant BT_RCVMTU   => 13;

# Bluetooth Protocols
use constant BTPROTO_L2CAP    => 0;
use constant BTPROTO_HCI      => 1;
use constant BTPROTO_SCO      => 2;
use constant BTPROTO_RFCOMM   => 3;
use constant BTPROTO_BNEP     => 4;
use constant BTPROTO_CMTP     => 5;
use constant BTPROTO_HIDP     => 6;
use constant BTPROTO_AVDTP    => 7;

# Bluetooth Socket Options
use constant SOL_HCI          => 0;
use constant SOL_L2CAP        => 6;
use constant SOL_SCO          => 17;
use constant SOL_RFCOMM       => 18;

# Bluetooth Socket Options
use constant SOL_BLUETOOTH    => 274;
use constant BDADDR_BREDR     => 0x00;
use constant BDADDR_LE_PUBLIC => 0x01;
use constant BDADDR_LE_RANDOM => 0x02;
use constant BDADDR_ANY       => "\0\0\0\0\0\0";
use constant BDADDR_ALL       => "\255\255\255\255\255\255";
use constant BDADDR_LOCAL     => "\0\0\0\255\255\255";

# L2CAP constants
use constant L2CAP_OPTIONS    => 0x01;
use constant L2CAP_CID_ATT    => 0x04;
use constant L2CAP_CID_SIG    => 0x05;
use constant L2CAP_PSM_SDP    => 0x0001;


sub new {
    my ($class, $cfg) = @_;
    return bless {_outbuffer => "", _outboxbuffer => "", _inboxbuffer => "", cfg => $cfg}, ref($class)||$class;
}

sub init {
    my ($self) = @_;
    $self->{_log_info} = "[".($self->{cfg}{b}||"no_bt").",key:".($self->{cfg}{k}||'<no>')."]";
    logger::info("Initializing BLE uart handler for $self->{_log_info}");
    my ($r_btaddr, $key, $l_btaddr) = ($self->{cfg}{b}, $self->{cfg}{k}, $self->{cfg}{l}{bt_listen_addr});
    my $l_addr = pack_sockaddr_bt(bt_aton($l_btaddr//BDADDR_ANY), 0);

    # new sockets, bind and request the right type for our bluetooth BLE UART
    # connection
    socket(my $s, AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP)
        // die "socket create problem: $!\n";
    my $fd = fileno($s);
    # set to non blocking mode now, and binmode
    my $c_info = "$self->{_log_info} (fd: $fd)";
    fcntl($s, F_SETFL, O_RDWR|O_NONBLOCK)
        // die "socket non-blocking set problem $c_info: $!\n";
    binmode($s)
        // die "binmode problem $c_info: $!\n";

    # bind
    bind($s, $l_addr)
        // die "$!\n";
    setsockopt($s, SOL_BLUETOOTH, BT_SECURITY, pack("S", BT_SECURITY_LOW))
        // die "setsockopt problem $c_info: $!\n";
    my @l2cap_opts;
    $l2cap_opts[0] = 23;
    $l2cap_opts[1] = 23;
    my $s_packed = pack("SSSCCCS", @l2cap_opts);
    setsockopt($s, SOL_BLUETOOTH, BT_RCVMTU, pack("CC", 0xA0, 0x02))
        // logger::error("setsockopt problem: $!");

    # now connect
    my $r_addr = pack_sockaddr_bt(bt_aton($r_btaddr), 0, L2CAP_CID_ATT, BDADDR_LE_PUBLIC);
    connect($s, $r_addr)
        // ($!{EINTR} or $!{EAGAIN} or $!{EINPROGRESS}) or die "problem connecting to $c_info: $!\n";

    # return info
    $self->{_socket} = $s;
    $self->{_fd}     = $fd;
    return $fd;
}

# see https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-54/out/en/host/attribute-protocol--att-.html

# GATT Primary Service Discovery (ATT Read By Group Type Request)
sub gatt_discovery_primary {
    my ($start_handle, $end_handle) = @_;
    $start_handle //= 0x0001;
    $end_handle   //= 0xFFFF;
    my $uuid = pack("S<", 0x2800); # 16-bit UUID for Primary Service
    return pack("CS<S<a*", 0x10, $start_handle, $end_handle, $uuid);
}

# GATT Secondary Service Discovery (ATT Read By Group Type Request)
sub gatt_discovery_secondary {
    my ($start_handle, $end_handle) = @_;
    $start_handle //= 0x0001;
    $end_handle   //= 0xFFFF;
    my $uuid = pack("S<", 0x2801); # 16-bit UUID for Secondary Service
    return pack("CS<S<a*", 0x10, $start_handle, $end_handle, $uuid);
}

# GATT Characteristic Discovery (ATT Read By Type Request)
sub gatt_char_discovery {
    my ($start_handle, $end_handle) = @_;
    my $uuid = pack("S<", 0x2803); # 16-bit UUID for Characteristic Declaration
    return pack("CS<S<a*", 0x08, $start_handle, $end_handle, $uuid);
}

# GATT Enable Notification (ATT Write Request to CCCD)
sub gatt_enable_notify {
    my ($cccd_handle) = @_;
    return pack("CS<S<", 0x12, $cccd_handle, 1);
}

# GATT Descriptor Discovery (ATT Find Information Request)
sub gatt_desc_discovery {
    my ($start_handle, $end_handle) = @_;
    return pack("CS<S<", 0x04, $start_handle, $end_handle);
}

# GATT MTU Request (ATT Exchange MTU Request)
sub gatt_mtu_request {
    my ($mtu) = @_;
    $mtu //= 23; # default MTU size
    return pack("CS<", 0x02, $mtu);
}

# GATT Write Request (ATT Write Request)
sub gatt_write {
    my ($handle, $value) = @_;
    return pack("CS<a*", 0x12, $handle, $value);
}

sub cleanup {
    my ($self) = @_;
    close($self->{_socket}) if defined $self->{_socket};
    delete $self->{_socket};
    delete $self->{_fd};
    substr($self->{_outbuffer} //="", 0, length($self->{_outbuffer}//"")) = "";
    substr($self->{_outboxbuffer} //="", 0, length($self->{_outboxbuffer}//"")) = "";
    return;
}

sub pack_sockaddr_bt {
    my ($bt_addr, $l2cap_port, $v1, $v2) = @_;
    return pack "SSa6SS", AF_BLUETOOTH, $l2cap_port, $bt_addr, $v1//L2CAP_CID_ATT, $v2//BDADDR_LE_PUBLIC;
}

sub bt_aton {
    return scalar reverse pack("H12", ($_[0]//BDADDR_ANY) =~ s/://gr);
}

sub pack_sockaddr_bt_rfcomm {
    my ($bt_addr, $v1) = @_;
    return pack "Sa6S", AF_BLUETOOTH, $bt_addr, $v1;
}

sub need_write {
    my ($self) = @_;

    # GATT state change/check/handle

    # State machine for GATT discovery and usage
    $self->{_gatt_state} //= 'mtu';

    # If we are in 'ready' state, check if we have a RX handle, and send data if we have data
    if($self->{_gatt_state} eq 'ready' and $self->{_nus_rx_handle}){
        logger::debug(sprintf "NUS ready, RX handle: 0x%04X, TX handle: 0x%04X", $self->{_nus_rx_handle}//0, $self->{_nus_tx_handle}//0);
        # If we have a RX handle, check if there is data in the outbox buffer
        if(length($self->{_outboxbuffer}//"")){
            # is there a RX handle set?
            my $r = index($self->{_outboxbuffer}, "\n");
            if ($r != -1) {
                my $_out = substr($self->{_outboxbuffer}, 0, $r + 1);
                # massage the buffer so a \n becomes a \r\n
                # this is only needed for AT command mode, note that if \n is already preceded with \r, it will not be changed
                $_out =~ s/\r?\n$/\r\n/ if $self->{cfg}{l}{uart_at} // 0;
                logger::debug(">>OUTBOX>>$_out>>".length($_out)." bytes to write to NUS (after massage): ".join('', map {sprintf '%02x', ord} split //, $_out));

                if(length($_out) > $self->{_att_mtu}){
                    logger::error("Data to write to NUS is too long: ".length($_out)." bytes, max is $self->{_att_mtu} bytes");
                } else {
                    logger::debug("Data to write to NUS is within MTU limits: ".length($_out)." bytes");

                    # append to the outbuffer
                    # this is the buffer that will be written to the socket
                    # it is not written immediately, but only when the socket is ready
                    # to write
                    my $ble_data = gatt_write($self->{_nus_rx_handle}, $_out);
                    if(defined $ble_data and length($ble_data) > 0){
                        substr($self->{_outboxbuffer}, 0, $r + 1, '');
                        logger::debug(">>BLE DATA>>".length($ble_data)." bytes to write to NUS");
                        $self->{_outbuffer} .= $ble_data;
                    } else {
                        logger::error("Problem packing data for NUS write");
                    }
                }
            } else {
                logger::debug("No newline in outbox buffer, waiting for more data");
            }
        }
    } elsif($self->{_gatt_state} eq 'mtu') {
        $self->{_gatt_state} = 'mtu_sent';
        # Request the ATT MTU size from the server
        logger::info("Requesting ATT MTU size from server: 256");
        $self->{_outbuffer} .= gatt_mtu_request(256);
    } elsif($self->{_gatt_state} eq 'desc_discovery') {
        $self->{_gatt_state} = 'desc_discovery_sent';
        # Start descriptor discovery for NUS RX characteristic
        logger::info(sprintf "Starting GATT Descriptor Discovery for NUS TX Characteristic (handle=0x%04X)", $self->{_nus_tx_handle});
        $self->{_outbuffer} .= gatt_desc_discovery($self->{_nus_tx_handle}+1, 0xFFFF);
    } elsif($self->{_gatt_state} eq 'notify_tx' and defined $self->{_nus_cccd}) {
        # Enable notifications on TX using discovered CCCD
        logger::info(sprintf "Enabling notifications for NUS TX Characteristic (handle=0x%04X)", $self->{_nus_cccd});
        $self->{_gatt_state} = 'notify_tx_sent';
        $self->{_outbuffer} .= gatt_enable_notify($self->{_nus_cccd});
    } elsif($self->{_gatt_state} eq 'char'){
        # If we have the service handles, start discovery of characteristics
        logger::info(sprintf "Starting GATT Characteristic Discovery for NUS service (start=0x%04X, end=0x%04X)", $self->{_char_start_handle}, $self->{_char_end_handle});
        $self->{_gatt_state} = 'char_discovery_sent';
        $self->{_outbuffer} .= gatt_char_discovery($self->{_char_start_handle}, $self->{_char_end_handle});
    } elsif($self->{_gatt_state} eq 'service') {
        logger::info("Sending GATT discovery request for primary services");
        $self->{_gatt_state} = 'service_discovery_sent';
        $self->{_outbuffer} .= gatt_discovery_primary($self->{_service_start_handle}, $self->{_service_end_handle});
    } else {
        # If we are not in a state where we can write, return 0
        logger::debug("No data to write, current GATT state: $self->{_gatt_state}, outbuffer length: ".length($self->{_outbuffer}//""));
    }

    logger::debug("Current outbuffer length: ".length($self->{_outbuffer}//""));
    return 1 if length($self->{_outbuffer}//"");
    return 0;
}

sub need_timeout {
    my ($self) = @_;
    return;
}

sub do_read {
    my ($self, $response) = @_;
    $response //= \(my $_d = '');
    my $data = "";
    my $r_sz = 512;
    while($::DATA_LOOP){
        my $r = sysread($self->{_socket}, $data, $r_sz);
        if(defined $r){
            # EOF?
            return 0 if $r == 0;
            local $!;
            my $r_data = $self->handle_ble_response_data($data);
            $$response .= $r_data if defined $r_data;
            $data = "";
        } else {
            return 1 if $!{EINTR} or $!{EAGAIN};
            die "problem reading data $self->{_log_info}: $!\n" if $!;
        }
    }
    return 1;
}

sub do_write {
    my ($self) = @_;
    my $n = length($self->{_outbuffer});
    logger::debug(">>WRITE>>$n>>".join('', map {sprintf '%02X', ord} split //, $self->{_outbuffer}));
    my $w = syswrite($self->{_socket}, $self->{_outbuffer}, $n, 0);
    if(defined $w){
        if($n == $w){
            substr($self->{_outbuffer}, 0, $n, '');
        } else {
            substr($self->{_outbuffer}, 0, $w, '');
        }
    } else {
        return if $!{EINTR} or $!{EAGAIN};
        die "problem writing data $self->{_log_info}: $!\n" if $!;
    }
    return;
}

sub handle_ble_response_data {
    my ($self, $data) = @_;
    return unless defined $data && length $data;

    my $opcode = unpack('C', $data);
    my $hex = join('', map {sprintf '%02X', ord($_)} split //, $data);
    logger::debug(sprintf "<<GATT<< opcode=0x%02X data=[%s]", $opcode, $hex);

    # State machine for GATT discovery and usage
    $self->{_gatt_state} //= 'mtu';

    if ($opcode == 0x11) { # Read By Group Type Response (Service Discovery)
        # Format: opcode(1) | length(1) | [handle(2) end_handle(2) uuid(2/16)]*
        my ($len) = unpack('xC', $data);
        my $count = (length($data) - 2) / $len;
        logger::info(sprintf "Service Discovery Response: %d services, entry len=%d", $count, $len);
        my $last_end = 0;
        for (my $i = 0; $i < $count; $i++) {
            my $entry = substr($data, 2 + $i * $len, $len);
            my ($start, $end, $uuid_raw) = unpack('S<S<a*', $entry);
            $last_end = $end if $end > $last_end;

            # Handle UUIDs
            $uuid_raw = reverse $uuid_raw if length($uuid_raw) == 16; # reverse for 16-byte UUIDs
            my $uuid;
            if (length($uuid_raw) == 2) {
                $uuid = uc(unpack('H*', $uuid_raw));
            } elsif (length($uuid_raw) == 16) {
                $uuid = uc(unpack('H*', $uuid_raw));
            } else {
                $uuid = join('', map { sprintf '%02X', ord($_) } split //, $uuid_raw);
            }
            logger::info(sprintf "  Service: start=0x%04X end=0x%04X uuid=0x%s", $start, $end, $uuid);

            # Compare against NUS UUID (normalize both to uppercase, no dashes)
            if (length($uuid) == 32 && lc($uuid) eq lc(NUS_SERVICE_UUID) =~ s/-//gr){
                logger::info(sprintf "Found NUS service: start=0x%04X end=0x%04X", $start, $end);
                $self->{_gatt_state} = 'char';
                $self->{_char_start_handle} = $start;
                $self->{_char_end_handle}   = $end;
                return;
            }
        }
        # If there may be more services, continue discovery
        if ($last_end && $last_end < 0xFFFF) {
            $self->{_gatt_state} = 'service';
            $self->{_service_start_handle} = $last_end + 1;
            $self->{_service_end_handle}   = 0xFFFF; # Continue until end
            return;
        }
    } elsif ($opcode == 0x03) { # ATT Server receive MTU size
        my ($mtu) = unpack('xS<', $data);
        logger::info(sprintf "ATT Server MTU size: %d bytes", $mtu);
        $self->{_att_mtu} = $mtu;
        # Set the initial state to 'service' to start service discovery
        $self->{_gatt_state} = 'service';
        $self->{_service_start_handle} = 0x0001;
        $self->{_service_end_handle}   = 0xFFFF;
        return;
    } elsif ($opcode == 0x09) { # Read By Type Response (Characteristic Discovery)
        my ($len) = unpack('xC', $data);
        my $count = (length($data) - 2) / $len;
        my $last_val_handle = 0;
        for (my $i = 0; $i < $count; $i++) {
            # Format: opcode(1) length(1) handle(2) properties(1) value_handle(2) uuid(2/16)
            my $entry = substr($data, 2 + $i * $len, $len);
            # Format: handle(2) properties(1) value_handle(2) uuid(2/16)
            my ($handle, $props, $val_handle, $uuid_raw) = unpack('S<CS<a*', $entry);

            $last_val_handle = $val_handle if $val_handle > $last_val_handle;

            # Handle UUIDs
            $uuid_raw = reverse $uuid_raw if length($uuid_raw) == 16; # reverse for 16-byte UUIDs
            my $uuid = lc join('', map {sprintf '%02X', ord($_)} split //, $uuid_raw);
            logger::info(sprintf "  Char: handle=0x%04X val_handle=0x%04X uuid=0x%s", $handle, $val_handle, $uuid);
            if ($uuid eq lc(NUS_RX_CHAR_UUID) =~ s/-//gr) {
                $self->{_nus_rx_handle} = $val_handle;
            } elsif ($uuid eq lc(NUS_TX_CHAR_UUID) =~ s/-//gr) {
                $self->{_nus_tx_handle} = $val_handle;
            }
            if($self->{_nus_rx_handle} && $self->{_nus_tx_handle}) {
                logger::info(sprintf "Found NUS characteristics: RX=0x%04X TX=0x%04X", $self->{_nus_rx_handle}, $self->{_nus_tx_handle});
                $self->{_gatt_state} = 'desc_discovery';
                return;
            }
        }
        # Continue discovery if not all characteristics are retrieved
        if (defined $self->{_char_end_handle} && $last_val_handle && $last_val_handle < $self->{_char_end_handle}) {
            $self->{_gatt_state} = 'char';
            $self->{_char_start_handle} = $last_val_handle + 1;
            $self->{_char_end_handle}   = $self->{_char_end_handle} // 0xFFFF;
            return;
        }
    } elsif ($opcode == 0x13) { # Write Response (for enabling notifications)
        logger::debug("GATT Write Response received, state: $self->{_gatt_state}");
        if ($self->{_gatt_state} eq 'notify_tx_sent') {
            $self->{_gatt_state} = 'ready';
            logger::debug(sprintf "NUS ready: RX=0x%04X TX=0x%04X", $self->{_nus_rx_handle}//0, $self->{_nus_tx_handle}//0);
        }
    } elsif ($opcode == 0x1b) { # Handle Value Notification
        my ($handle) = unpack('xS<', $data);
        my $value = substr($data, 3);
        if ($handle == $self->{_nus_tx_handle}) {
            logger::debug("NUS RX Notification: ".$value);
            $self->{_inboxbuffer} .= $value;
            while($self->{_inboxbuffer} =~ s/^((.*?)\r?\n)//){
                my $line = $2;
                logger::debug("Received NUS data: $line, ".length($line)." bytes: ".join('', map {sprintf '%02X', ord($_)} split //, $line));
                # Process the line as needed, e.g., print or store
                # return for AT command mode
                return $line;
            }
        }
    } elsif ($opcode == 0x01) { # Error Response
        my ($req_opcode, $handle, $err_code) = unpack('xCS<C', $data);
        # map the error code to a human-readable message
        # this is not exhaustive, but covers common cases
        my $err_msg;
        if( $err_code == 0x01) {
            $err_msg = "Invalid Handle";
        } elsif ($err_code == 0x02) {
            $err_msg = "Read Not Permitted";
        } elsif ($err_code == 0x03) {
            $err_msg = "Write Not Permitted";
        } elsif ($err_code == 0x04) {
            $err_msg = "Invalid PDU";
        } elsif ($err_code == 0x05) {
            $err_msg = "Insufficient Authentication";
        } elsif ($err_code == 0x06) {
            $err_msg = "Request Not Supported";
        } elsif ($err_code == 0x07) {
            $err_msg = "Invalid Offset";
        } elsif ($err_code == 0x08) {
            $err_msg = "Insufficient Authorization";
        } elsif ($err_code == 0x09) {
            $err_msg = "Prepare Queue Full";
        } elsif ($err_code == 0x0A) {
            $err_msg = "Attribute Not Found";
        } elsif ($err_code == 0x0B) {
            $err_msg = "Attribute Not Long";
        } elsif ($err_code == 0x0C) {
            $err_msg = "Insufficient Encryption Key Size";
        } elsif ($err_code == 0x0D) {
            $err_msg = "Invalid Attribute Value Length";
        } elsif ($err_code == 0x0E) {
            $err_msg = "Unlikely Error";
        } elsif ($err_code == 0x0F) {
            $err_msg = "Insufficient Encryption";
        } elsif ($err_code == 0x10) {
            $err_msg = "Unsupported Group Type";
        } elsif ($err_code == 0x11) {
            $err_msg = "Insufficient Resources";
        } elsif ($err_code == 0x12) {
            $err_msg = "Application Error";
        } elsif ($err_code == 0x13) {
            $err_msg = "Attribute Not Found (GATT)";
        } elsif ($err_code == 0x14) {
            $err_msg = "Attribute Not Long (GATT)";
        } elsif ($err_code >= 0x15 and $err_code <= 0x9F) {
            $err_msg = sprintf("Reserved Error Code: 0x%02X", $err_code);
        } elsif ($err_code >= 0xE0 and $err_code <= 0xFF) {
            $err_msg = sprintf("Vendor Specific Error Code: 0x%02X", $err_code);
        } else {
            $err_msg = sprintf("Unknown Error Code: 0x%02X", $err_code);
        }
        logger::error(sprintf "ATT Error Response: req_opcode=0x%02X handle=0x%04X code=0x%02X msg=%s", $req_opcode, $handle, $err_code, $err_msg);

    } elsif ($opcode == 0x05) { # Find Information Response (Descriptor Discovery)
        # Parse descriptors, look for CCCD (0x2902)
        my ($fmt) = unpack('xC', $data); # 0x01 = 16-bit UUID, 0x02 = 128-bit UUID
        my $entry_len = $fmt == 1 ? 4 : 18;
        my $count = (length($data) - 2) / $entry_len;
        for (my $i = 0; $i < $count; $i++) {
            my $entry = substr($data, 2 + $i * $entry_len, $entry_len);
            my ($handle, $uuid_raw);
            if ($fmt == 1) {
                ($handle, $uuid_raw) = unpack('S<S<', $entry);
                $uuid_raw = pack('S<', $uuid_raw);
            } else {
                ($handle, $uuid_raw) = unpack('S<a16', $entry);
            }
            my $uuid = lc(unpack('H*', reverse $uuid_raw));
            logger::info(sprintf "Descriptor: handle=0x%04X uuid=0x%s", $handle, $uuid);
            # CCCD UUID: 00002902-0000-1000-8000-00805f9b34fb, or 2902 in 16-bit format
            if (lc($uuid) =~ /^2902$/) {
                $self->{_nus_cccd} = $handle;
                logger::info(sprintf "Found NUS TX CCCD: handle=0x%04X", $handle);
                $self->{_gatt_state} = 'notify_tx';
                return;
            }
        }
    } else {
        logger::info(sprintf "Unhandled GATT/ATT opcode: 0x%02X", $opcode);
    }
    return;
}

package utils;

use strict; use warnings;

sub load_cpan {
    my ($c) = @_;
    eval "require $c";
    return $c unless $@;
    return;
}

sub usage {
    my (%msg) = @_;
    no warnings 'once';
    utils::load_cpan("Pod::Usage");
    local $ENV{PAGER} = $ENV{PAGER}||$::ENV{PAGER}||"less";
    local $0 = $::DOLLAR_ZERO // $0;
    FindBin->again();
    Pod::Usage::pod2usage(
        -input   => "$FindBin::Bin/$FindBin::Script",
        -exitval => 1,
        -output  => '>&STDERR',
        %msg
    );
    return;
}

package logger;

use strict; use warnings;
no warnings 'once';

use POSIX ();
use Time::HiRes ();

our $_json_printer;
our $_init_syslog;
our $_default_loglevel;

*log_fatal = *fatal;
*log_error = *error;
*log_info  = *info;
*log_debug = *debug;

sub fatal {
    my (@msg) = @_;
    my $msg = do_log("error", @msg);
    die "$msg\n";
}

sub error {
    my (@msg) = @_;
    return unless lc(utils::cfg("loglevel", $_default_loglevel//="info")) =~ m/^(error|info|debug)$/
                    || utils::cfg("DEBUG", 0);
    return do_log("error", @msg);
}

sub info {
    my (@msg) = @_;
    return unless lc(utils::cfg("loglevel", $_default_loglevel//="info")) =~ m/^(info|debug)$/
                    || utils::cfg("DEBUG", 0);
    return do_log("info", @msg);
}

sub debug {
    my (@msg) = @_;
    return unless lc(utils::cfg("loglevel", $_default_loglevel//="info")) =~ m/^(debug)$/
                    || utils::cfg("DEBUG", 0);
    no warnings 'once';
    my @r_msg = eval {
        require Data::Dumper;
        local $Data::Dumper::Sortkeys = 1;
        local $Data::Dumper::Indent   = 0;
        local $Data::Dumper::Terse    = 1;
        local $Data::Dumper::Deepcopy = 1;
        return do_log("debug", map {ref($_)?Data::Dumper::Dumper($_):$_} @msg);
    };
    if($@){
        chomp(my $err = $@);
        logger::error("problem logging debug message: $err");
        return;
    }
    return @r_msg;
}

sub do_log {
    my ($w, @msg) = @_;
    local $::LOG_PREFIX = $::LOG_PREFIX // "";
    @msg =
        map {split m/\n/, $_//""}
        join("",
        map {defined $_ and ref($_)
            ?do {
                $_ = eval {
                    require JSON;
                    $_json_printer //= JSON->new->canonical->allow_nonref->allow_unknown->allow_blessed->convert_blessed->allow_tags->indent(0);
                    $_json_printer->encode($_);
                };
                $_//"";
            }
            :$_//""
        } @msg);
    my ($tm, $usec) = Time::HiRes::gettimeofday();
    $usec = sprintf("%06d", $usec);
    my @tm = gmtime($tm);
    my $msg = join("\n", map {POSIX::strftime("%H:%M:%S.$usec", @tm)." [$$] [$w]: $::LOG_PREFIX$_"} map {split m/\n/, $_//""} @msg);
    print STDERR "$msg\n";
    return $msg;
}

package utils;

use strict; use warnings;

sub cfg {
    my ($k, $default_v, $nm, $do_exception, $r) = @_;
    no warnings 'once';
    $nm //= $::APP_MODULE // "";
    my $env_k_m = ($::APP_NAME?$::APP_NAME."_":"")."${nm}_$k";
    my $env_k_a = ($::APP_NAME?$::APP_NAME."_":"")."$k";
    my $v = ($r and UNIVERSAL::can($r, "variable") and $r->variable(lc($env_k_a)))
        // $::APP_ENV{uc($env_k_m) =~ s/\W/_/gr}
        // $::APP_ENV{uc($env_k_a) =~ s/\W/_/gr}
        // $::APP_CFG{$env_k_m}
        // $::APP_CFG{$env_k_a}
        // $::APP_CFG{$k}
        // $ENV{uc($env_k_m) =~ s/\W/_/gr}
        // $ENV{uc($env_k_a) =~ s/\W/_/gr}
        // $default_v;
    die "need '$k' config or $env_k_m/$env_k_a ENV variable\n"
        if $do_exception and not defined $v;
    return $v;
}

sub set_cfg {
    my ($k, $v) = @_;
    my $env_k_a = uc(($::APP_NAME?$::APP_NAME."_":"")."$k") =~ s/\W/_/gr;
    $::APP_CFG{$env_k_a} = $v;
    return $v;
}

=head1 NAME

ble_uart.pl - BLE UART (Nordic UART Service) bridge in Perl

=head1 SYNOPSIS

    perl ble_uart.pl [key1]=XX:XX:XX:XX:XX:XX[,uart_at=1]

=head1 ARGUMENTS

The script expects one or more Bluetooth addresses of BLE devices implementing
the Nordic UART Service (NUS). Each address can be prefixed with a key
to identify the device, and can include additional options like `uart_at=1`
to enable UART AT command mode. The format is:

    [key]=XX:XX:XX:XX:XX:XX[,option=value ...]

Where:

=over 4

=item key
An optional key to identify the device, which can be used in logs.

=item XX:XX:XX:XX:XX:XX
The Bluetooth address of the device to connect to.

=item option
An optional configuration option, such as `uart_at=1` to enable UART AT command mode.

=back

=head1 OPTIONS

=head2 Command Line Options

    --loglevel=N      Set log verbosity (default: info)
    --help            Show help
    --man             Show full manual

=head1 DESCRIPTION

This script connects to a BLE device implementing the Nordic UART Service (NUS),
discovers the UART RX/TX characteristics, and allows simple UART-style read/write
over BLE. It is intended for use with ESP32/ESP-AT or similar BLE UART bridges.

Input from STDIN is sent to the BLE device, and data received from the device is
printed to STDOUT.

=head1 AUTHOR

CowboyTim

=cut
