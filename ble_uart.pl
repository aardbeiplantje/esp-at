# GATT Characteristic Discovery (ATT Read By Type Request)
sub gatt_char_discovery {
    my ($start_handle, $end_handle) = @_;
    my $opcode = 0x08;
    my $uuid = pack("S<", 0x2803); # 16-bit UUID for Characteristic Declaration
    return pack("CS<S<a*", $opcode, $start_handle, $end_handle, $uuid);
}

# GATT Enable Notification (ATT Write Request to CCCD)
sub gatt_enable_notify {
    my ($cccd_handle) = @_;
    my $value = pack("S<", 0x0001); # notifications enabled
    return gatt_write($cccd_handle, $value);
}
#!/usr/bin/perl

use strict; use warnings;

no warnings 'once';

use FindBin;
use Getopt::Long;
use Errno qw(EAGAIN EINTR);
use POSIX ();
use List::Util ();

BEGIN {
    my $timezone = POSIX::tzname();
    $ENV{TZ} = readlink('/etc/localtime') =~ s|/usr/share/zoneinfo/||gr;
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

    # line buffered STDOUT for now, as we print data instead of logging into files
    select STDOUT; $| = 1; select STDOUT;

    # our clean exiting
    $::APP::LOOP = 1;
    my $exit_handler_sub = sub {
        $::APP::LOOP = 0; die "$_[0] signal, exiting\n"
    };
    local $SIG{INT}  = $exit_handler_sub;
    local $SIG{TERM} = $exit_handler_sub;

    # we start non-sleepy
    my $s_timeout = 0;

    # initialize our targets
    connect_tgt($connections, $_) for @{$tgts};

    eval {
    while($::APP::LOOP){
        # reset select() vecs
        $rin = "";
        $win = "";
        $ein = "";

        my ($m_curl_r, $m_curl_w, $m_curl_e);
        my $m_curl_had_fds = 0;

        # select() vec handling
        my @shuffled_conns = List::Util::shuffle(sort keys %{$connections});
        foreach my $fd (@shuffled_conns){
            my $c = $connections->{$fd};
            logger::debug($c);
            # always read
            vec($rin, $fd, 1) = 1;

            # other stuff to write?
            vec($win, $fd, 1) = $c->need_write();

            # need select() timeout
            my $tm = $c->need_timeout();
            $s_timeout = $tm if defined $tm and !defined $s_timeout and $tm <= ($s_timeout//86400);
        }
        $s_timeout = 0 if defined $s_timeout and $s_timeout < 0;

        # Add STDIN to read set for tty input
        my $stdin_fd = fileno(STDIN);
        vec($rin, $stdin_fd, 1) = 1;

        # get the main multi curl fd set to be added for our select
        if(defined $$::MAIN_CURL){
            (my $m_err, $m_curl_r, $m_curl_w, $m_curl_e) = http::request_fdset($::MAIN_CURL);
            vec($rin, $_, 1) = 1 for @$m_curl_r;
            vec($win, $_, 1) = 1 for @$m_curl_w;
            vec($ein, $_, 1) = 1 for @$m_curl_e;
            $m_curl_had_fds = 1 if @$m_curl_r or @$m_curl_w or @$m_curl_e;
            if(!$m_curl_had_fds){
                my $m_timeout = http::request_timeout($::MAIN_CURL);
                $s_timeout = $m_timeout if defined $m_timeout and $m_timeout < $s_timeout;
            }
        }

        # our main loop
        logger::info("will TIMEOUT: ".($s_timeout//"not"))
            if ($::LOGLEVEL//0) >= 7;
        $ein |= $rin | $win;
        my $r = select(my $rout = $rin, my $wout = $win, my $eout = $ein, $s_timeout);
        if($r == -1){
            $!{EINTR} or $!{EAGAIN} or logger::error("select problem: $!");
            next;
        }

        # handle STDIN/tty input
        if(vec($rout, $stdin_fd, 1)) {
            my $inbuf = '';
            my $n = sysread(STDIN, $inbuf, 1024);
            if(defined $n && $n > 0) {
                # send to the target with "uart AT" as config, and only thr first
                foreach my $c (values %{$connections}) {
                    next unless $c->{cfg}{l}{uart_at} // 0;
                    # massage the buffer so a \n becomes a \r\n
                    $inbuf =~ s/\n/\r\n/g;
                    # If NUS RX handle is known, send as GATT write
                    if ($c->{_nus_rx_handle}) {
                        $c->{_outbuffer} = gatt_write($c->{_nus_rx_handle}, $inbuf);
                    } else {
                        $c->{_outbuffer} //= '';
                        $c->{_outbuffer} .= $inbuf;
                    }
                    last;
                }
            } elsif(defined $n && $n == 0) {
                # EOF on STDIN, exit loop
                $::APP::LOOP = 0;
                last;
            }
        }

        # anything to read?
        foreach my $fd (@shuffled_conns){
            my $c = $connections->{$fd};
            eval {
                # check error
                if(vec($eout, $fd, 1)){
                    die "Error select: bad FD $fd\n";
                }

                # handle read if select says so
                if(vec($rout, $fd, 1)){
                    my $read_ok = $c->do_read();
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
                do {$::APP::LOOP = 0; last} if $err =~ m/^QUIT at .* line \d+/;
                logger::error($err);
                removing_tgt($connections, $c);
            }
        }

        # handle multi curl fdset changes
        if($m_curl_had_fds and defined $$::MAIN_CURL){
            # now check the fdset for the main curl, and if a change, run perform
            my $changed = 0;
            $changed |= vec($rout, $_, 1) for @$m_curl_r;
            $changed |= vec($wout, $_, 1) for @$m_curl_w;
            $changed |= vec($eout, $_, 1) for @$m_curl_e;
            if($changed){
                eval {http::request_perform($::MAIN_CURL)};
                if($@){
                    chomp(my $err = $@);
                    logger::error($err);
                }
            }
        }

        # make new connections
        if(keys %{$connections} != @{$tgts}){
            foreach my $t (@{$tgts}){
                next if grep {$t->{b} eq $_->{b}} values %{$connections};
                eval {connect_tgt($connections, $t)};
                if($@){
                    chomp(my $err = $@);
                    do {$::APP::LOOP = 0; last} if $err =~ m/^QUIT at .* line \d+/;
                    logger::error("problem reconnecting [$t->{b},key:$t->{k}]: $err");
                }
            }
        }

        # next select timeout?
        $s_timeout = (keys %{$connections} != @{$tgts})?1:undef;
    }
    };
    chomp(my $err = $@);

    # handle clean exits
    removing_tgt($connections, $_) for values %{$connections};

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

package ble::uart;

use strict; use warnings;

use Errno qw(EAGAIN EINTR EINPROGRESS);
use Fcntl qw(F_SETFL O_RDWR O_NONBLOCK);
use Socket;


use constant NUS_SERVICE_UUID => "6e400001b5a3f393e0a9e50e24dcca9e";
use constant NUS_RX_CHAR_UUID => "6e400002b5a3f393e0a9e50e24dcca9e";
use constant NUS_TX_CHAR_UUID => "6e400003b5a3f393e0a9e50e24dcca9e";

# constants for BLUETOOTH that come from bluez

use constant AF_BLUETOOTH     => 31;
use constant PF_BLUETOOTH     => 31;

use constant BT_SECURITY        => 4;
use constant BT_SECURITY_SDP    => 0;
use constant BT_SECURITY_LOW    => 1;
use constant BT_SECURITY_MEDIUM => 2;
use constant BT_SECURITY_HIGH   => 3;
use constant BT_SECURITY_FIPS   => 4;
use constant BT_SNDMTU   => 12;
use constant BT_RCVMTU   => 13;

use constant BTPROTO_L2CAP    => 0;
use constant BTPROTO_HCI      => 1;
use constant BTPROTO_SCO      => 2;
use constant BTPROTO_RFCOMM   => 3;
use constant BTPROTO_BNEP     => 4;
use constant BTPROTO_CMTP     => 5;
use constant BTPROTO_HIDP     => 6;
use constant BTPROTO_AVDTP    => 7;

use constant SOL_HCI          => 0;
use constant SOL_L2CAP        => 6;
use constant SOL_SCO          => 17;
use constant SOL_RFCOMM       => 18;

use constant SOL_BLUETOOTH    => 274;
use constant BDADDR_BREDR     => 0x00;
use constant BDADDR_LE_PUBLIC => 0x01;
use constant BDADDR_LE_RANDOM => 0x02;
use constant BDADDR_ANY       => "\0\0\0\0\0\0";
use constant BDADDR_ALL       => "\255\255\255\255\255\255";
use constant BDADDR_LOCAL     => "\0\0\0\255\255\255";

use constant L2CAP_OPTIONS    => 0x01;
use constant L2CAP_CID_ATT    => 0x04;
use constant L2CAP_CID_SIG    => 0x05;
use constant L2CAP_PSM_SDP    => 0x0001;


sub new {
    my ($class, $cfg) = @_;
    return bless {cfg => $cfg}, ref($class)||$class;
}

sub init {
    my ($self) = @_;
    $self->{_log_info} = "[".($self->{cfg}{b}||"no_bt").",key:".($self->{cfg}{k}||'<no>')."]";
    logger::info("initializing BLE uart handler for $self->{_log_info}");
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


# GATT Primary Service Discovery (ATT Read By Group Type Request)
# opcode: 0x10, start handle: 0x0001, end handle: 0xFFFF, group type: 0x2800
sub gatt_discovery_primary {
    my $opcode = 0x10;
    my $start_handle = 0x0001;
    my $end_handle = 0xFFFF;
    my $uuid = pack("S<", 0x2800); # 16-bit UUID for Primary Service
    return pack("CS<S<a*", $opcode, $start_handle, $end_handle, $uuid);
}

# GATT Secondary Service Discovery (ATT Read By Group Type Request)
# opcode: 0x10, start handle: 0x0001, end handle: 0xFFFF, group type: 0x2801
# This is not used in NUS, but can be added if needed
sub gatt_discovery_secondary {
    my $opcode = 0x10;
    my $start_handle = 0x0001;
    my $end_handle = 0xFFFF;
    my $uuid = pack("S<", 0x2801); # 16-bit UUID for Secondary Service
    return pack("CS<S<a*", $opcode, $start_handle, $end_handle, $uuid);
}


# GATT Write Request (ATT Write Request)
# opcode: 0x12, handle: 2 bytes, value: variable
sub gatt_write {
    my ($handle, $value) = @_;
    my $opcode = 0x12;
    $value //= "";
    return pack("CS<a*", $opcode, $handle, $value);
}

sub cleanup {
    my ($self) = @_;
    close($self->{_socket}) if defined $self->{_socket};
    delete $self->{_socket};
    delete $self->{_fd};
    delete $self->{_sent_request};
    delete $self->{_outbuffer};
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
    return 1 if length($self->{_outbuffer}//"");
    return 0 if exists $self->{_sent_request};
    $self->{_sent_request} = 1;
    $self->{_outbuffer} = gatt_discovery_primary();
    return 1;
}

sub need_timeout {
    my ($self) = @_;
    return;
}

sub do_read {
    my ($self) = @_;
    my $data = "";
    my $r_sz = 512;
    while($::APP::LOOP){
        my $r = sysread($self->{_socket}, $data, $r_sz);
        if(defined $r){
            # EOF?
            return 0 if $r == 0;
            local $!;
            $self->handle_ble_response_data($data);
            $data = "";
            #return 1 if $r < 512;
        } else {
            return 1 if $!{EINTR} or $!{EAGAIN};
            die "problem reading data $self->{_log_info}: $!\n" if $!;
        }
    }
    return 1;
}

sub do_write {
    my ($self) = @_;
    return unless defined $self->{_outbuffer};
    my $n = length($self->{_outbuffer});
    logger::debug(">>WRITE>>$n>>".join('', map {sprintf '%04X', ord} split //, $self->{_outbuffer}));
    my $w = syswrite($self->{_socket}, $self->{_outbuffer}, $n, 0);
    if(defined $w){
        if($n == $w){
            delete $self->{_outbuffer};
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
    my $hex = join(' ', map { sprintf '%02X', ord($_) } split //, $data);
    logger::debug(sprintf "<<GATT<< opcode=0x%02X data=[%s]", $opcode, $hex);

    # State machine for GATT discovery and usage
    $self->{_gatt_state} //= 'service';

    if ($opcode == 0x11) { # Read By Group Type Response (Service Discovery)
        # Format: opcode(1) | length(1) | [handle(2) end_handle(2) uuid(2/16)]*
        my ($len) = unpack('xC', $data);
        my $count = (length($data) - 2) / $len;
        logger::info(sprintf "Service Discovery Response: %d services, entry len=%d", $count, $len);
        my $last_end = 0;
        for (my $i = 0; $i < $count; $i++) {
            my $entry = substr($data, 2 + $i * $len, $len);
            my ($start, $end, $uuid_raw) = unpack('S>S>a*', $entry);
            $last_end = $end if $end > $last_end;
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
            my $nus_uuid = uc(NUS_SERVICE_UUID);
            $nus_uuid =~ s/-//g;
            if (length($uuid) == 32 && $uuid eq $nus_uuid) {
                $self->{_nus_start} = $start;
                $self->{_nus_end} = $end;
                $self->{_gatt_state} = 'char';
                $self->{_outbuffer} = gatt_char_discovery($start, $end);
                return;
            }
        }
        # If there may be more services, continue discovery
        if ($last_end && $last_end < 0xFFFF) {
            my $opcode = 0x10;
            my $start_handle = $last_end + 1;
            my $end_handle = 0xFFFF;
            my $uuid = pack("S<", 0x2800);
            $self->{_outbuffer} = pack("CS<S<a*", $opcode, $start_handle, $end_handle, $uuid);
            return;
        }
    } elsif ($opcode == 0x09) { # Read By Type Response (Characteristic Discovery)
        my ($len) = unpack('xC', $data);
        my $count = (length($data) - 2) / $len;
        for (my $i = 0; $i < $count; $i++) {
            my $entry = substr($data, 2 + $i * $len, $len);
            my ($handle, $props, $val_handle, $uuid) = unpack('S> C S> a*', $entry);
            $uuid = join('', map { sprintf '%02X', ord($_) } split //, $uuid);
            logger::info(sprintf "  Char: handle=0x%04X val_handle=0x%04X uuid=0x%s", $handle, $val_handle, $uuid);
            if (lc($uuid) eq lc(NUS_RX_CHAR_UUID)) {
                $self->{_nus_rx_handle} = $val_handle;
            } elsif (lc($uuid) eq lc(NUS_TX_CHAR_UUID)) {
                $self->{_nus_tx_handle} = $val_handle;
                $self->{_nus_tx_cccd} = $val_handle + 1; # CCCD is next handle
            }
        }
        # If both handles found, enable notifications on TX
        if ($self->{_nus_tx_cccd}) {
            $self->{_gatt_state} = 'notify';
            $self->{_outbuffer} = gatt_enable_notify($self->{_nus_tx_cccd});
            return;
        }
    } elsif ($opcode == 0x13) { # Write Response (for enabling notifications)
        if ($self->{_gatt_state} eq 'notify') {
            $self->{_gatt_state} = 'ready';
            logger::info("NUS ready: RX=0x".sprintf('%04X',$self->{_nus_rx_handle})." TX=0x".sprintf('%04X',$self->{_nus_tx_handle}));
        }
    } elsif ($opcode == 0x1b) { # Handle Value Notification
        my ($handle) = unpack('xS>', $data);
        my $value = substr($data, 3);
        if ($handle == $self->{_nus_tx_handle}) {
            logger::info("NUS TX Notification: ".$value);
            print STDOUT $value;
        }
    } elsif ($opcode == 0x01) { # Error Response
        my ($req_opcode, $handle, $err_code) = unpack('xC S> C', $data);
        logger::error(sprintf "ATT Error Response: req_opcode=0x%02X handle=0x%04X code=0x%02X", $req_opcode, $handle, $err_code);
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
    my $env_k_a = ($::APP_NAME?$::APP_NAME."_":"")."$k";
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
