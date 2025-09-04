#!/usr/bin/perl
#
# ble_uart.pl - BLE UART (Nordic UART Service) bridge in Perl
#
# This script connects to one or more BLE devices implementing the Nordic UART
# Service (NUS), providing a terminal interface for interactive communication
# over Bluetooth Low Energy. Supports multiple simultaneous connections,
# scripting, and AT command mode for ESP32/ESP-AT devices.
#
# This is free and unencumbered software released into the public domain.
#
# Anyone is free to copy, modify, publish, use, compile, sell, or
# distribute this software, either in source code form or as a compiled
# binary, for any purpose, commercial or non-commercial, and by any
# means.
#
# In jurisdictions that recognize copyright laws, the author or authors
# of this software dedicate any and all copyright interest in the
# software to the public domain. We make this dedication for the benefit
# of the public at large and to the detriment of our heirs and
# successors. We intend this dedication to be an overt act of
# relinquishment in perpetuity of all present and future rights to this
# software under copyright law.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# For more information, please refer to <https://unlicense.org>
#

BEGIN {
    $ENV{LC_ALL} = "C";
};

use strict; use warnings;

no warnings 'once';
use utf8;

BEGIN {
    $::APP_NAME = "ble:uart";
};

use FindBin;
use Errno qw(EAGAIN EINTR);

BEGIN {
    $::DOLLAR_ZERO = $0;
    $0 = $::APP_NAME;
};

# app/loop state and config, a global instance variable
$::APP_OPTS = handle_cmdline_options();

# start the application
eval {
    main_loop();
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

    # for possible  socket problems, although we do non blocking very fast
    local $SIG{PIPE} = "IGNORE";
    # makes syscalls restarted
    local $SIG{HUP}  = "IGNORE";

    # init global constants
    $::APP_CONN = {};
    $::CURRENT_CONNECTION = undef;
    $::COMMAND_BUFFER     = undef;

    # our clean exiting
    $::DATA_LOOP = 1;
    my $exit_handler_sub = sub {
        $::DATA_LOOP = 0; die "$_[0] signal, exiting\n"
    };
    local $SIG{INT}  = $exit_handler_sub;
    local $SIG{TERM} = $exit_handler_sub;

    # we start non-sleepy
    my $s_timeout;

    # select vecs
    my ($rin, $win, $ein);

    # initialize our targets
    my $tgts = $::APP_OPTS->{targets} // [];
    logger::debug("starting main loop with targets", $tgts);
    connect_tgt($::APP_CONN, $_, 1) for @{$tgts};

    # assume the current connection is the first one if we have one
    $::CURRENT_CONNECTION = (grep {$_->{cfg}{b} eq $tgts->[0]{b}} values %{$::APP_CONN})[0]
        if @{$tgts} and keys %{$::APP_CONN};

    # our input reader - choose between TTY and STDIN based on whether STDIN is a terminal
    my $reader = (-t STDIN and !utils::cfg('raw')) ? input::tty->new() : input::stdin->new();
    my $response_buffer = "";
    my $color_ok = $::APP_OPTS->{_color_ok};
    my $c_reset = $colors::reset_color;
    $c_reset = "" unless $color_ok;
    my ($e_color, $s_color) = ("", "");
    $e_color = $colors::bright_red_color    if $color_ok;
    $s_color = $colors::bright_yellow_color if $color_ok;

    # a message printer subroutine, different for STDIN vs TTY, and use
    # closures for less cpu cycle use (reuse precomputed stuff and store it)
    my $msg_printer;
    if(utils::cfg('raw')){
        $msg_printer = sub {
            my ($data_ref) = @_;
            my $r = syswrite($reader->outfh(), $$data_ref);
            if(!defined $r){
                logger::error("syswrite error: $!");
                $::DATA_LOOP = 0; # exit the main loop
                return;
            }
            return;
        };
    } else {
        my $ttydisplaybuffer = "";
        $msg_printer = sub {
            my ($data_ref) = @_;
            return unless length($$data_ref//"");
            logger::debug(">>TTY>> showing message, length:".length($$data_ref));

            # utf8 handling
            if($::APP_OPTS->{_utf8_ok}){
                require Encode;
                Encode::_utf8_on($$data_ref);
            }
            $ttydisplaybuffer //= "";
            $ttydisplaybuffer  .= $$data_ref;

            # remote info
            my $b_addr = "";
            $b_addr = $::APP_OPTS->{_utf8_ok} ? "❰$::CURRENT_CONNECTION->{cfg}{b}❱❱ " : "[$::CURRENT_CONNECTION->{cfg}{b}] "
                if defined $::CURRENT_CONNECTION;
            $b_addr = $colors::dark_yellow_color.$b_addr if $color_ok and length($b_addr);

            # command info
            my $c_info = "";
            $c_info = $::COMMAND_BUFFER // "";
            $::COMMAND_BUFFER = undef;
            chomp($c_info);
            $c_info = " ($c_info)" if length($c_info);
            $c_info = $colors::bright_blue_color3.$c_info if $color_ok;

            while($ttydisplaybuffer =~ s/(.*?\n)//){
                my $l = $1;
                last unless length($l//"");
                $l =~ s/\r?\n$//;
                my $c_resp  = $l =~ m/^\+ERROR:/ ? $e_color : $s_color;
                $c_resp = "" unless $color_ok;
                $reader->show_message($::APP_OPTS->{_reply_line_prefix}.$b_addr.$c_resp.$l.$c_info.$c_reset);
            }
            return;
        };
    }

    # shuffler sub? note that "keys" in perl randomizes already since perl 5.18
    my $shuffler_sub = sub {@_};
    if(utils::cfg("shuffle_connections")){
        utils::load_cpan("List::Util");
        $shuffler_sub = \&List::Util::shuffle;
    }

    $::OUTBOX = [];
    # If --script was given, run it before entering the main loop
    handle_command("/script $::APP_OPTS->{script}") if $::APP_OPTS->{script};

    # main loop
    eval {
    while($::DATA_LOOP){

        # check all connections, and if they have an empty outbox, exit
        if($::DATA_LOOP_EXIT_WANTED){
            my $all_empty = 1;
            foreach my $c (values %{$::APP_CONN}){
                logger::debug("checking connection $c->{_log_info} (fd: $c->{_fd}), buffer: ", length($c->{_outboxbuffer}//""), " bytes");
                if(length($c->{_outboxbuffer}//"") > 0){
                    $all_empty = 0;
                    last;
                }
            }
            if($all_empty and !defined $::COMMAND_BUFFER and !@{$::OUTBOX}){
                logger::info("all connections have empty outbox, exiting");
                $::DATA_LOOP = 0;
                last;
            }
        }

        # reset select() vecs each loop iteration
        $rin = "";
        $win = "";
        $ein = "";

        # input, note: STDIN is FD=0, so use defined check here
        vec($rin, $reader->infd(), 1) = 1 if defined $reader->infd() and !defined $::COMMAND_BUFFER;

        # select() vec handling
        foreach my $fd (&{$shuffler_sub}(keys %{$::APP_CONN})){
            logger::debug("checking connection", $fd);
            my $c = $::APP_CONN->{$fd};
            vec($rin, $fd, 1) = 1;

            # other stuff to write?
            vec($win, $fd, 1) = $c->need_write();

            # need select() timeout
            my $tm = $c->need_timeout();
            $s_timeout = $tm if defined $tm and (!defined $s_timeout or (defined $s_timeout and $tm < $s_timeout));
        }

        # next select timeout? if we are missing connections, set to 1s, else undef (wait forever)
        my $n_conns = (keys %{$::APP_CONN} != @{$tgts}) ? 1 : undef;
        $s_timeout = $n_conns if !defined $s_timeout or (defined $s_timeout and defined $n_conns and $n_conns < $s_timeout);
        $s_timeout = 0 if defined $s_timeout and $s_timeout < 0;

        # reader timeout?
        $s_timeout = 0 if @{$::OUTBOX} and !defined $::COMMAND_BUFFER and !defined $s_timeout;

        # we're waiting for a command response? timeout short to run the spinner
        $s_timeout = 0.1 if defined $::COMMAND_BUFFER and defined $s_timeout and $s_timeout > 0.1;
        logger::debug(">>TTY>> setting select timeout to", $s_timeout);

        # do select() call, waiting for changes
        $ein |= $rin | $win;
        my $r = select(my $rout = $rin, my $wout = $win, my $eout = $ein, $s_timeout);
        if($r == -1){
            $!{EINTR} or $!{EAGAIN} or logger::error("select problem: $!");
            next;
        }

        # check IN|OUT for reader
        if(vec($rout, $reader->infd(), 1)){
            $reader->do_read();
        }

        # process reader's OUTBOX messages if there are any
        if(!defined $::COMMAND_BUFFER and defined(my $cmd_data = shift @{$::OUTBOX})){
            logger::debug(">>TTY>>", length($cmd_data), " bytes read from TTY");
            unless(handle_command($cmd_data)){
                if(defined $::CURRENT_CONNECTION){
                    $::CURRENT_CONNECTION->{_outboxbuffer} .= $cmd_data;
                    $::COMMAND_BUFFER = $cmd_data;
                } else {
                    $reader->show_message("${e_color}No current connection set, cannot send data$c_reset\n");
                }
            }
        }

        # anything to read from the remote connections?
        foreach my $fd (&{$shuffler_sub}(keys %{$::APP_CONN})){
            my $c = $::APP_CONN->{$fd};
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
                        removing_tgt($::APP_CONN, $c);
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
                removing_tgt($::APP_CONN, $c);
            }
        }

        # make new connections
        if(keys %{$::APP_CONN} != @{$tgts}){
            foreach my $t (@{$tgts}){
                next if grep {$t->{b} eq $_->{b}} values %{$::APP_CONN};
                eval {connect_tgt($::APP_CONN, $t)};
                if($@){
                    chomp(my $err = $@);
                    do {$::DATA_LOOP = 0; last} if $err =~ m/^QUIT at .* line \d+/;
                    logger::error("problem reconnecting [$t->{b}]: $err");
                }
            }
        }

        # if we have a response buffer, write it to the TTY
        if(length($response_buffer) > 0){
            my $resp = substr($response_buffer, 0, length($response_buffer), '');
            logger::debug(">>TTY>>", length($resp), " bytes to write to TTY");
            &{$msg_printer}(\$resp);
        }

        # if we have a COMMAND_BUFFER, but no response, we spin the prompt
        if(defined $::COMMAND_BUFFER and !length($response_buffer)){
            $reader->spin();
        }

        $s_timeout = undef; # reset timeout for next iteration
    }
    };
    chomp(my $err = $@);

    # handle clean exits
    removing_tgt($::APP_CONN, $_) for values %{$::APP_CONN};

    # cleanup reader
    $reader->cleanup();

    # relay error if any
    die "$err\n" if $err and $err !~ m/^(TERM|INT) signal, exiting$/;
    return
}

sub removing_tgt {
    my ($conns, $c) = @_;
    return unless defined $c and defined $c->{_fd};
    logger::info("cleanup $c->{_log_info} (fd: $c->{_fd})");
    delete $conns->{$c->{_fd}};
    $c->cleanup();
    return;
}

sub connect_tgt {
    my ($conns, $c, $blocking) = @_;
    my $n = ble::uart->new({%$c});
    my $fd = $n->init($blocking);
    return unless defined $fd;
    $n->blocking(0);
    $conns->{$fd} = $n;
    return;
}

sub handle_cmdline_options {
    my $cfg = {};

    if(!utils::cfg('raw') or grep {/^--?(raw|r|manpage|man|m|help|h|\?|script)$/} @ARGV){
        require Getopt::Long;
        Getopt::Long::GetOptions(
            $cfg,
            "raw|r!",
            "manpage|man|m!",
            "help|h|?!",
            "script=s",
        ) or utils::usage(-exitval => 1);
        utils::usage(-verbose => 1, -exitval => 0)
            if $cfg->{help};
        utils::manpage(1)
            if $cfg->{manpage};
        utils::set_cfg($_, $cfg->{$_}) for qw(raw);
    }

    $cfg->{_reply_line_prefix} = "";
    if(utils::cfg("raw")){
        utils::set_cfg('loglevel', 'NONE') unless defined utils::cfg('loglevel');
    } else {
        # color support?
        if(utils::cfg("interactive_color", 1)){
            $cfg->{_color_ok} = 1 if ($ENV{COLORTERM}//"") =~ /color/i or ($ENV{TERM}//"") =~ /color/i;
        }

        # UTF-8 support?
        $cfg->{_utf8_ok} = 1 if utils::cfg('interactive_utf8', 1);
        $cfg->{_reply_line_prefix} = $cfg->{_utf8_ok} ? "↳ " : "> ";
        if($cfg->{_color_ok}){
            $cfg->{_reply_line_prefix} = $colors::green_color.$cfg->{_reply_line_prefix}.$colors::reset_color;
        }
    }

    # parse the cmdline options for targets to connect to
    $cfg->{targets} = [];
    add_target($cfg, $_) for @ARGV;
    return $cfg;
}

sub add_target {
    my ($cfg, $tgt) = @_;
    my ($addr, $opts) = ($tgt//"") =~ m/^(..:..:..:..:..:..)(?:,(.*))?$/;
    unless ($addr) {
        print "usage: /connect XX:XX:XX:XX:XX:XX[,option=value]\n";
        return 0;
    }
    foreach my $t (@{$cfg->{targets}}) {
        if ($t->{b} eq $addr) {
            print "Already configured target: $addr\n";
            return 0;
        }
    }
    # test connection
    eval {
        my $bc = ble::uart->new({b => $addr, l => $opts});
        my $fd = $bc->init(1);
        if(!defined $fd){
            die "Failed to connect to $addr\n";
        }
        $bc->blocking(0);
        $::CURRENT_CONNECTION = $::APP_CONN->{$fd} = $bc;
    };
    if($@){
        chomp(my $err = $@);
        print "Failed to connect to $addr: $err\n";
        return 0;
    }
    logger::info("Adding target: $addr");
    # adding the target
    push @{$cfg->{targets}}, {
        b => $addr,
        l => {split m/=|,/, $opts//""}
    };
    return 1;
}

our @cmds;
BEGIN {
    @cmds = qw(/exit /quit /disconnect /connect /help /debug /logging /loglevel /man /usage /switch /script);
};

sub handle_command {
    my ($line) = @_;
    chomp($line);
    $line =~ s/^\s+//; $line =~ s/\s+$//;
    return 0 if $line eq '' || $line =~ /^#/;

    logger::debug("command", $line);
    if ($line =~ m|^/exit| or $line =~ m|^/quit|) {
        $::DATA_LOOP_EXIT_WANTED = 1;
        return 1;
    }
    if ($line =~ m|^/man|) {
        utils::manpage(0);
        return 1;
    }
    if ($line =~ m|^/usage|) {
        utils::usage(-verbose => 1, -exitval => 'NOEXIT');
        return 1;
    }
    if ($line =~ m|^/connect\s*(\S+)?|) {
        main::add_target($::APP_OPTS, $1);
        return 1;
    }
    if ($line =~ m|^/disconnect\s*(.*)|) {
        my $tgt = $1 // '';
        if(!@{$::APP_OPTS->{targets}}) {
            print "No targets configured to disconnect.\n";
            return 1;
        }
        if($tgt){
            $tgt =~ s/^\s+|\s+$//g; # trim whitespace
            my $found = 0;
            foreach my $c (sort keys %{$::APP_CONN}) {
                if ($::APP_CONN->{$c}->{cfg}{b} eq $tgt) {
                    main::removing_tgt($::APP_CONN, $::APP_CONN->{$c});
                    $::COMMAND_BUFFER = undef;
                    $::CURRENT_CONNECTION = undef;
                    $found = 1;
                    last;
                }
            }
            foreach my $t (@{$::APP_OPTS->{targets}}) {
                if ($t->{b} eq $tgt) {
                    @{$::APP_OPTS->{targets}} = grep {$_->{b} ne $tgt} @{$::APP_OPTS->{targets}};
                    last;
                }
            }
            if (!$found) {
                print "No target found with address: $tgt\n";
            } else {
                print "Disconnected target: $tgt\n";
            }
            return 1;
        } else {
            $::COMMAND_BUFFER = undef;
            $::CURRENT_CONNECTION = undef;
            @{$::APP_OPTS->{targets}} = ();
            foreach my $c (sort keys %{$::APP_CONN}){
                (delete $::APP_CONN->{$c})->cleanup();
            }
            print "Disconnected all BLE connections.\n";
        }
        return 1;
    }
    if ($line =~ m|^/script\s+(\S+)|) {
        my $file = $1;
        unless (-r $file) {
            print "Cannot read script file: $file\n";
            return 1;
        }
        open(my $fh, '<', $file) or do {print "Failed to open $file: $!\n"; return 1;};
        print "Executing script: $file\n";
        my $r_code = 0;
        while (my $cmd = <$fh>) {
            chomp $cmd;
            $cmd =~ s/^\s+//; $cmd =~ s/\s+$//;
            next if $cmd eq '' || $cmd =~ /^#/;
            print "> $cmd\n";
            my $r = handle_command($cmd);
            if(!$r) {
                # data to be sent to the current connection
                logger::debug("Adding command to outbox:", $cmd);
                push @{$::OUTBOX}, "$cmd\n";
            }
        }
        close $fh;
        return 1;
    }
    if ($line =~ m|^/debug\s*(on\|off)?|) {
        my $arg = $1 // '';
        if ($arg eq 'on') {
            utils::cfg('loglevel', 'DEBUG');
            utils::cfg('debug', 1);
            print "Debugging enabled (loglevel=DEBUG)\n";
        } elsif ($arg eq 'off') {
            utils::cfg('loglevel', 'NONE');
            utils::cfg('debug', 0);
            print "Debugging disabled (loglevel=INFO)\n";
        } else {
            print "Usage: /debug on|off\n";
        }
        return 1;
    }
    if ($line =~ m|^/logging\s*(on\|off)?|) {
        my $arg = $1 // '';
        if ($arg eq 'on') {
            utils::cfg('loglevel', 'INFO');
            print "Logging enabled (loglevel=INFO)\n";
        } elsif ($arg eq 'off') {
            utils::cfg('loglevel', 'NONE');
            print "Logging disabled (loglevel=NONE)\n";
        } else {
            print "Usage: /logging on|off\n";
        }
        return 1;
    }
    if ($line =~ m|^/loglevel\s*(none\|info\|warn\|error\|debug)?|) {
        my $lvl = $1;
        if (defined $lvl) {
            utils::cfg('loglevel', uc($lvl));
            print "Log level set to $lvl\n";
        } else {
            print "Usage: /loglevel <none|info|warn|error|debug>\n";
        }
        return 1;
    }
    if ($line =~ m|^/help|) {
        print join(", ", @main::cmds)."\n";
        return 1;
    }
    if ($line =~ m|^/switch\s*(\S+)?|) {
        my $tgt = $1 // '';
        if (!$tgt) {
            print "Usage: /switch <BTADDR>\n";
            print "Connected devices:\n";
            foreach my $c (values %{$::APP_CONN}) {
                print "  $c->{cfg}{b}\n";
            }
            return 1;
        }
        my $found = 0;
        foreach my $c (values %{$::APP_CONN}) {
            if ($c->{cfg}{b} eq $tgt) {
                $::CURRENT_CONNECTION = $c;
                print "Switched to device: $tgt\n";
                $found = 1;
                last;
            }
        }
        print "No connected device with address: $tgt\n" unless $found;
        return 1;
    }
    if ($line =~ m|^/|) {
        print "Unknown command: $line\n";
        return 1;
    }
    return;
}


package input::tty;

use strict; use warnings;

our $BASE_DIR;
our $HISTORY_FILE;

BEGIN {
    $BASE_DIR     //= $ENV{BLE_UART_DIR}
                  // ($ENV{HOME}//"/home/$ENV{LOGNAME}")."/.ble_uart";
    $HISTORY_FILE //= $ENV{BLE_UART_HISTORY_FILE}
                  // "${BASE_DIR}_history";
};

BEGIN {
package colors;

our $red_color           = "\033[0;31m";
our $bright_red_color    = "\033[38;5;196;1m";
our $green_color         = "\033[0;32m";
our $dark_green_color    = "\033[38;5;28;1m";
our $yellow_color1       = "\033[0;33m";
our $dark_yellow_color   = "\033[38;5;136;1m";
our $bright_yellow_color = "\033[38;5;226;1m";
our $blue_color1         = "\033[0;34m";
our $bright_blue_color1  = "\033[38;5;21;1m";
our $blue_color2         = "\033[38;5;25;1m";
our $bright_blue_color2  = "\033[38;5;33;1m";
our $blue_color3         = "\033[38;5;13;1m";
our $bright_blue_color3  = "\033[38;5;39;1m";
our $magenta_color       = "\033[0;35m";
our $cyan_color          = "\033[0;36m";
our $white_color         = "\033[0;37m";
our $reset_color         = "\033[0m";
};

sub new {
    my ($class, $cfg) = @_;
    $cfg //= {};
    my $self = bless {%$cfg}, ref($class)||$class;

    local $ENV{PERL_RL}   = 'Gnu';
    local $ENV{TERM}      = $ENV{TERM} // 'vt220';
    local $ENV{COLORTERM} = $ENV{COLORTERM} // 'truecolor';
    eval {require Term::ReadLine; require Term::ReadLine::Gnu};
    if($@){
        logger::error("Please install Term::ReadLine and Term::ReadLine::Gnu\n\nE.g.:\n  sudo apt install libterm-readline-gnu-perl");
        exit 1;
    }
    my $term = Term::ReadLine->new("aicli");
    $term->read_init_file("$BASE_DIR/inputrc");
    $term->ReadLine('Term::ReadLine::Gnu') eq 'Term::ReadLine::Gnu'
        or die "Term::ReadLine::Gnu is required\n";

    $term->using_history();
    $term->ReadHistory($HISTORY_FILE);
    $term->clear_signals();
    my $attribs = $term->Attribs();
    $attribs->{attempted_completion_function} = \&chat_word_completions_cli;
    $attribs->{ignore_completion_duplicates} = 1;
    $attribs->{catch_signals} = 0;
    $attribs->{catch_sigwinch} = 0;
    my ($t_ps1, $t_ps2) = $self->create_prompt();
    $term->callback_handler_install(
        $t_ps1,
        sub {
            my ($n_ps1, $n_ps2) = $self->create_prompt();
            $self->rl_cb_handler($n_ps1, $n_ps2, @_);
            return;
        }
    );
    $term->save_prompt();
    $term->clear_message();
    $term->message(($::APP_OPTS->{_color_ok}?$colors::bright_red_color:"")."Welcome to the BLE UART CLI".($::APP_OPTS->{_color_ok}?$colors::reset_color:""));
    $term->crlf();
    $term->restore_prompt();
    $term->save_prompt();
    $term->clear_message();
    $term->message(($::APP_OPTS->{_color_ok}?$colors::bright_yellow_color:"")."Type /help for available commands, /usage for usage doc".($::APP_OPTS->{_color_ok}?$colors::reset_color:""));
    $term->crlf();
    $term->restore_prompt();
    $term->on_new_line();
    $term->redisplay();
    $self->{_rl} = $term;
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
    return $self->{_rl}->callback_read_char();
}

sub show_message {
    my ($self, $m) = @_;
    my ($n_ps1, $ps2) = $self->create_prompt();
    my $t = $self->{_rl};
    $t->set_prompt($n_ps1);
    $t->save_prompt();
    $t->clear_message();
    $t->message($m);
    $t->crlf();
    $t->restore_prompt();
    $t->on_new_line_with_prompt();
    $t->redisplay();
    return;
}

sub spin {
    my ($self) = @_;
    $self->{_spinners}  //= $::APP_OPTS->{_utf8_ok} ? [qw(⠋ ⠙ ⠹ ⠸ ⠼ ⠴ ⠦ ⠧)] : [qw(- \ | /)];
    $self->{_spin_pos}  //= 0;
    $self->{_spin_icon} //= do {
        my $s_icon;
        if($::APP_OPTS->{_color_ok}){
            $s_icon = $::APP_OPTS->{_utf8_ok}
            ? " ".$colors::red_color.'⌛'.$colors::bright_red_color." Waiting for response...".$colors::reset_color
            : $colors::bright_red_color." Waiting for response...".$colors::reset_color;
        } else {
            $s_icon = $::APP_OPTS->{_utf8_ok}
            ? " ⌛ Waiting for response..."
            : " Waiting for response...";
        }
        $s_icon;
    };
    $self->{_spinner_color} = $::APP_OPTS->{_color_ok} ? $colors::bright_red_color : "";
    my $t = $self->{_rl};
    $t->save_prompt();
    $t->clear_message();
    $t->message($self->{_spinner_color}.$self->{_spinners}[$self->{_spin_pos}].$self->{_spin_icon});
    $t->restore_prompt();
    $t->on_new_line_with_prompt();
    $self->{_spin_pos}++;
    $self->{_spin_pos} = 0 if $self->{_spin_pos} >= @{$self->{_spinners}};
    return;
}

sub chat_word_completions_cli {
    my ($text, $line, $start, $end) = @_;
    $line =~ s/ +$//g;
    my @rcs = ();
    my @wrd = split m/\s+/, $line, -1;
    logger::debug("W: >", @wrd, "<\n");
    foreach my $w (@wrd) {
        next unless $w =~ m|^/|;
        foreach my $k (@main::cmds) {
            push @rcs, $k if !index($k, $w) or $k eq $w;
        }
    }
    logger::debug("R: >", @rcs, "<");
    return '', @rcs;
}

sub cleanup {
    my ($self) = @_;
    my $t = $self->{_rl};
    $t->callback_handler_remove();
    return;
}

sub rl_cb_handler {
    my ($self, $t_ps1, $t_ps2, $line) = @_;
    my $t = $self->{_rl};
    if(!defined $line){
        $t->callback_handler_remove();
        $::DATA_LOOP = 0; # exit the main loop
        return;
    }
    $t->set_prompt($t_ps1);
    my $buf = \($self->{_buf} //= '');
    if($line !~ m/^$/ms){
        $line =~ s/^\s+//;
        $line =~ s/\s+$//;
        $line =~ s/\r?\n$//;
        logger::debug(">>TTY>>", length($line), " bytes read from TTY: $line");
        # not a command it OR we already had a buffer,
        if(utils::cfg('interactive_multiline', 0)){
            # multiline input, we need to buffer it

            # do readline stuff
            $t->set_prompt($t_ps2);

            # add to buffer until we have an empty line entered
            $$buf .= "$line\n";
            return;
        } else {
            # just process the line
            logger::debug(">>TTY>> processing line:", $line);

            # do readline stuff
            $t->addhistory($line);
            $t->WriteHistory($HISTORY_FILE);
            $t->set_prompt($t_ps1);
            $t->on_new_line_with_prompt();
            $t->redisplay();

            # add to the output buffer
            $$buf .= "$line\n";
            push @{$::OUTBOX}, $$buf;
            $$buf = '';
            return;
        }
        return;
    } else {
        logger::debug(">>TTY>> empty line read from TTY, processing buffer");
        # empty line, this is command execution if we have a buffer
        if(length($$buf)){
            logger::debug("BUF: >>", $$buf, "<<");
            $t->addhistory($$buf);
            $t->WriteHistory($HISTORY_FILE);
            $t->set_prompt($t_ps1);
            $t->on_new_line_with_prompt();
            $t->redisplay();
            push @{$::OUTBOX}, $$buf;
            $$buf = '';
        }
        return;
    }
    return;
}

sub create_prompt {
    my ($self) = @_;
    # https://jafrog.com/2013/11/23/colors-in-terminal.html
    # https://ss64.com/bash/syntax-colors.html
    # If a BLE device is connected, show its address in yellow
    my $ble_addr = $::CURRENT_CONNECTION ? $::CURRENT_CONNECTION->{cfg}{b} : undef;
    my $PR = "";
    if($ble_addr){
        $PR = $::APP_OPTS->{_utf8_ok} ? "❲$ble_addr❳" : "|$ble_addr|";
        if($::APP_OPTS->{_color_ok}) {
            $PR  = $colors::yellow_color1.$PR.$colors::reset_color;
            $PR .= $colors::bright_blue_color2."AT".$colors::reset_color;
        } else {
            $PR .= "AT";
        }
    }
    my $PP1 = $::APP_OPTS->{_utf8_ok} ? "► " : "> ";
    my $PP2 = $::APP_OPTS->{_utf8_ok} ? "│ " : "| ";
    if($::APP_OPTS->{_color_ok}) {
        $PP1 = $colors::bright_blue_color2.$PP1.$colors::reset_color;
        $PP2 = $colors::bright_blue_color3.$PP2.$colors::reset_color;
    }
    my $prompt_term1 = "$PR$PP1";
    my $prompt_term2 = "$PP2";
    if ($::APP_OPTS->{_color_ok}) {
        if ($ble_addr) {
            $prompt_term1 = $colors::green_color.$prompt_term1.$colors::reset_color;
        } else {
            $prompt_term1 = $colors::blue_color3.$prompt_term1.$colors::reset_color;
        }
        $prompt_term2 = $colors::blue_color3.$prompt_term2.$colors::reset_color;
    }
    my $ps1 = eval "return \"$prompt_term1\"" || $PP1;
    my $ps2 = eval "return \"$prompt_term2\"" || $PP2;
    return ($ps1, $ps2);
}

package input::stdin;

use strict; use warnings;

use Errno qw(EAGAIN EWOULDBLOCK);
use Fcntl qw(F_GETFL F_SETFL O_NONBLOCK);

sub new {
    my ($class, $cfg) = @_;
    $cfg //= {};
    my $self = bless {%$cfg}, ref($class)||$class;

    # Set STDIN to non-blocking mode
    my $flags = fcntl(STDIN, F_GETFL, 0)
        or die "Can't get flags for STDIN: $!\n";
    fcntl(STDIN, F_SETFL, $flags | O_NONBLOCK)
        or die "Can't set STDIN non-blocking: $!\n";

    $self->{_buffer} = "";
    return $self;
}

sub infd {
    my ($self) = @_;
    return fileno(STDIN);
}

sub infh {
    my ($self) = @_;
    return \*STDIN;
}

sub outfd {
    my ($self) = @_;
    return fileno(STDOUT);
}

sub outfh {
    my ($self) = @_;
    return \*STDOUT;
}

sub do_read {
    my ($self) = @_;
    my $r = sysread(STDIN, my $data, 1);
    if (!defined $r) {
        return if $!{EAGAIN} || $!{EWOULDBLOCK};
        die "Error reading from STDIN: $!\n";
    } elsif ($r == 0) {
        # EOF - signal main loop to exit
        $::DATA_LOOP = 0;
        return;
    }
    $self->{_buffer} .= $data;

    # Process complete lines
    while ($self->{_buffer} =~ s/^([^\r\n]*)\r?\n//) {
        my $line = $1;
        chomp $line;
        $line =~ s/^\s+//;
        $line =~ s/\s+$//;

        if (length($line) > 0) {
            logger::debug(">>STDIN>> processing line", $line);
            push @{$::OUTBOX}, "$line\n";
        }
    }
    return 1;
}

sub show_message {
    my ($self, $m) = @_;
    return unless length($m//"");
    no warnings 'utf8'; # disable utf8 warnings for STDIN mode
    print STDOUT $m;
    return;
}

sub spin {
    my ($self) = @_;
    # No spinner for STDIN mode
    return;
}

sub cleanup {
    my ($self) = @_;
    # Nothing special to clean up for STDIN
    return;
}

package ble::uart;

use strict; use warnings;

use Errno qw(EAGAIN EINTR EINPROGRESS EWOULDBLOCK);
use Fcntl qw(F_GETFL F_SETFL O_RDWR O_NONBLOCK);
use Socket;

# constants for BLE UART (Nordic UART Service) UUIDs - configurable via environment variables
our $NUS_SERVICE_UUID;
our $NUS_RX_CHAR_UUID;
our $NUS_TX_CHAR_UUID;
BEGIN {
    $NUS_SERVICE_UUID = $ENV{BLE_UART_NUS_SERVICE_UUID} // "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
    $NUS_RX_CHAR_UUID = $ENV{BLE_UART_NUS_RX_CHAR_UUID} // "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
    $NUS_TX_CHAR_UUID = $ENV{BLE_UART_NUS_TX_CHAR_UUID} // "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
};

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

our %err_msg;
BEGIN {
%err_msg = (
    0x01 => "Invalid Handle",
    0x02 => "Read Not Permitted",
    0x03 => "Write Not Permitted",
    0x04 => "Invalid PDU",
    0x05 => "Insufficient Authentication",
    0x06 => "Request Not Supported",
    0x07 => "Invalid Offset",
    0x08 => "Insufficient Authorization",
    0x09 => "Prepare Queue Full",
    0x0A => "Attribute Not Found",
    0x0B => "Attribute Not Long",
    0x0C => "Insufficient Encryption Key Size",
    0x0D => "Invalid Attribute Value Length",
    0x0E => "Unlikely Error",
    0x0F => "Insufficient Encryption",
    0x10 => "Unsupported Group Type",
    0x11 => "Insufficient Resources",
    0x12 => "Application Error",
    0x13 => "Attribute Not Found (GATT)",
    0x14 => "Attribute Not Long (GATT)",
);
};

sub new {
    my ($class, $cfg) = @_;
    return bless {_outbuffer => "", _outboxbuffer => "", cfg => $cfg}, ref($class)||$class;
}

sub init {
    my ($self, $blocking) = @_;
    $self->{_log_info} = "[".($self->{cfg}{b}||"no_bt")."]";
    logger::info("Initializing BLE uart handler for $self->{_log_info}");
    my ($r_btaddr, $l_btaddr) = ($self->{cfg}{b}, $self->{cfg}{l}{bt_listen_addr});
    if($l_btaddr){
        # check if the local bluetooth address is valid
        if($l_btaddr !~ m/^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$/){
            die "Invalid local bluetooth address: $l_btaddr\n";
        }
        # pack the local bluetooth address
        $l_btaddr = lc($l_btaddr);
        $l_btaddr =~ s/://g; # remove colons
        $l_btaddr = pack("H12", $l_btaddr);
        $l_btaddr = reverse $l_btaddr; # reverse the byte order
    } else {
        $l_btaddr = BDADDR_ANY; # use any local bluetooth address
    }
    my $l_addr = pack_sockaddr_bt(bt_aton($l_btaddr), 0);

    # new sockets, bind and request the right type for our bluetooth BLE UART
    # connection
    socket(my $s, AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP)
        // die "socket create problem: $!\n";
    my $fd = fileno($s);
    fcntl($s, F_GETFL, my $fcntl_flags = 0)
        // die "socket fcntl get problem: $!\n";
    $fcntl_flags |= O_RDWR; # set to read/write mode
    if($blocking){
        # set to blocking mode
        $fcntl_flags &= ~O_NONBLOCK;
    } else {
        # set to non-blocking mode
        $fcntl_flags |= O_NONBLOCK;
    }

    # set to non blocking mode now, and binmode
    my $c_info = "$self->{_log_info} (fd: $fd)";
    fcntl($s, F_SETFL, $fcntl_flags)
        // die "socket non-blocking set problem $c_info: $!\n";
    binmode($s)
        // die "binmode problem $c_info: $!\n";

    # bind
    bind($s, $l_addr)
        // die "bind error $c_info: $!\n";
    setsockopt($s, SOL_BLUETOOTH, BT_SECURITY, pack("S", BT_SECURITY_LOW))
        // die "setsockopt problem $c_info: $!\n";
    setsockopt($s, SOL_BLUETOOTH, BT_RCVMTU, pack("CC", 0xA0, 0x02))
        // logger::error("setsockopt problem: $!");

    # now connect
    my $r_addr = pack_sockaddr_bt(bt_aton($r_btaddr), 0, L2CAP_CID_ATT, BDADDR_LE_PUBLIC);
    connect($s, $r_addr)
        // ($!{EINTR} or $!{EAGAIN} or $!{EINPROGRESS})
        or die "problem connecting to $c_info: $!\n";

    # return info
    $self->{_socket} = $s;
    $self->{_fd}     = $fd;
    return $fd;
}

sub blocking {
    my ($self, $blocking) = @_;
    return unless defined $self->{_socket};
    return unless defined $blocking;

    fcntl($self->{_socket}, F_GETFL, my $fcntl_flags = 0)
        // die "socket fcntl get problem: $!\n";
    if($blocking){
        # set to blocking mode
        $fcntl_flags &= ~O_NONBLOCK;
    } else {
        # set to non-blocking mode
        $fcntl_flags |= O_NONBLOCK;
    }
    fcntl($self->{_socket}, F_SETFL, $fcntl_flags)
        // die "socket non-blocking set problem: $!\n";
    return;
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
                $_out =~ s/\r?\n$/\r\n/ if $self->{cfg}{l}{uart_at} // 1;
                logger::debug(">>OUTBOX>>", $_out, ">>", length($_out), " bytes to write to NUS (after massage): ", utils::tohex($_out));

                if(length($_out) > $self->{_att_mtu}){
                    logger::error("Data to write to NUS is too long: ", length($_out), " bytes, max is ", $self->{_att_mtu}, " bytes");
                } else {
                    logger::debug("Data to write to NUS is within MTU limits: ", length($_out), " bytes");

                    # append to the outbuffer
                    # this is the buffer that will be written to the socket
                    # it is not written immediately, but only when the socket is ready
                    # to write
                    my $ble_data = gatt_write($self->{_nus_rx_handle}, $_out);
                    if(defined $ble_data and length($ble_data) > 0){
                        substr($self->{_outboxbuffer}, 0, $r + 1, '');
                        logger::debug(">>BLE DATA>>", length($ble_data), " bytes to write to NUS");
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
        logger::debug("No data to write, current GATT state: ", $self->{_gatt_state}, ", outbuffer length: ", length($self->{_outbuffer}//""));
    }

    logger::debug("Current outbuffer length: ", length($self->{_outbuffer}//""));
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
            return 1 if $!{EINTR} or $!{EAGAIN} or $!{EWOULDBLOCK};
            die "problem reading data $self->{_log_info}: $!\n" if $!;
        }
    }
    return 1;
}

sub do_write {
    my ($self) = @_;
    my $n = length($self->{_outbuffer});
    logger::debug(">>WRITE>>", $n, ">>", utils::tohex($self->{_outbuffer}));
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
    logger::debug(sprintf "<<GATT<< opcode=0x%02X data=[%s]", $opcode, utils::tohex($data));

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
                $uuid = utils::tohex($uuid_raw);
            }
            logger::info(sprintf "  Service: start=0x%04X end=0x%04X uuid=0x%s", $start, $end, $uuid);

            # Compare against NUS UUID (normalize both to uppercase, no dashes)
            if (length($uuid) == 32 && lc($uuid) eq lc($NUS_SERVICE_UUID) =~ s/-//gr){
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
            my $uuid = lc(utils::tohex($uuid_raw));
            logger::info(sprintf "  Char: handle=0x%04X val_handle=0x%04X uuid=0x%s", $handle, $val_handle, $uuid);
            if ($uuid eq lc($NUS_RX_CHAR_UUID) =~ s/-//gr) {
                $self->{_nus_rx_handle} = $val_handle;
            } elsif ($uuid eq lc($NUS_TX_CHAR_UUID) =~ s/-//gr) {
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
        logger::debug("GATT Write Response received, state: ", $self->{_gatt_state});
        if ($self->{_gatt_state} eq 'notify_tx_sent') {
            $self->{_gatt_state} = 'ready';
            logger::debug(sprintf "NUS ready: RX=0x%04X TX=0x%04X", $self->{_nus_rx_handle}//0, $self->{_nus_tx_handle}//0);
        }
    } elsif ($opcode == 0x1b) { # Handle Value Notification
        my ($handle) = unpack('xS<', $data);
        my $value = substr($data, 3);
        if (($handle//0) == ($self->{_nus_tx_handle}//0)) {
            logger::debug("NUS RX Notification: ", length($value), " data: ", utils::tohex($value));
            return $value if length($value) > 0;
        }
    } elsif ($opcode == 0x01) { # Error Response
        my ($req_opcode, $handle, $err_code) = unpack('xCS<C', $data);
        # map the error code to a human-readable message
        # this is not exhaustive, but covers common cases
        my $emsg = $err_msg{$err_code};
        if(!defined $emsg){
            if ($err_code >= 0x15 and $err_code <= 0x9F) {
                $emsg = sprintf("Reserved Error Code: 0x%02X", $err_code);
            } elsif ($err_code >= 0xE0 and $err_code <= 0xFF) {
                $emsg = sprintf("Vendor Specific Error Code: 0x%02X", $err_code);
            } else {
                $emsg = sprintf("Unknown Error Code: 0x%02X", $err_code);
            }
        }
        logger::error(sprintf "ATT Error Response: req_opcode=0x%02X handle=0x%04X code=0x%02X msg=%s", $req_opcode, $handle, $err_code, $emsg);
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
    local $ENV{MANPAGER} = $ENV{MANPAGER}||$::ENV{MANPAGER}||"/usr/bin/less";
    local $ENV{PAGER}    = $ENV{PAGER}||$ENV{MANPAGER};
    my $p_fn = do {
        local $0 = $::DOLLAR_ZERO // $0;
        FindBin->again();
        "$FindBin::Bin/$FindBin::Script";
    };
    Pod::Usage::pod2usage(
        -input   => $p_fn,
        -exitval => 1,
        -output  => '>&STDERR',
        %msg
    );
    return;
}

sub manpage {
    my ($do_exit) = @_;
    # if both "pod2man" and "man" exists, use those, else, fallback to usage()
    my $ok_man = 1;
    system("which pod2man >/dev/null 2>&1 </dev/null") == 0 and
    system("which man     >/dev/null 2>&1 </dev/null") == 0 or do {$ok_man = 0};
    if($ok_man){
        local $ENV{MANPAGER} = $ENV{MANPAGER}||$::ENV{MANPAGER}||"less";
        my $p_fn = do {
            local $0 = $::DOLLAR_ZERO // $0;
            FindBin->again();
            "$FindBin::Bin/$FindBin::Script";
        };
        system("pod2man $p_fn|man /dev/stdin");
        exit 0 if $do_exit;
    } else {
        utils::usage(-verbose => 2, -exitval => 1, -output => undef);
    }
    return;
}

sub tohex {
    my ($data) = @_;
    return join('', map {sprintf '%02X', ord($_)} split //, $data);
}

package logger;

use strict; use warnings;
no warnings 'once';

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
    my $msg = join("\n", map {sprintf("%02d:%02d:%02d", reverse ((gmtime())[0,1,2]))." [$$] [$w]: $::LOG_PREFIX$_"} map {split m/\n/, $_//""} @msg);
    print STDERR "$msg\n";
    return $msg;
}

package utils;

use strict; use warnings;

sub cfg {
    my ($k, $default_v, $nm, $do_exception, $r) = @_;
    no warnings 'once';
    $nm //= $::APP_MODULE // "";
    my $env_k_m = uc(($::APP_NAME?$::APP_NAME."_":"")."${nm}_$k") =~ s/\W/_/gr;
    my $env_k_a = uc(($::APP_NAME?$::APP_NAME."_":"").$k) =~ s/\W/_/gr;
    my $v = ($r and UNIVERSAL::can($r, "variable") and $r->variable(lc($env_k_a)))
        // $::APP_ENV{$env_k_m}
        // $::APP_ENV{$env_k_a}
        // $::APP_CFG{$env_k_m}
        // $::APP_CFG{$env_k_a}
        // $ENV{$env_k_m}
        // $ENV{$env_k_a}
        // $default_v;
    die "need '$k' config or $env_k_m/$env_k_a ENV variable\n"
        if $do_exception and not defined $v;
    return $v;
}

sub set_cfg {
    my ($k, $v) = @_;
    my $env_k_a = uc(($::APP_NAME?$::APP_NAME."_":"").$k) =~ s/\W/_/gr;
    $::APP_CFG{$env_k_a} = $v;
    return $v;
}

__END__
=pod

=head1 NAME

ble_uart.pl - BLE UART (Nordic UART Service) bridge in Perl

=head1 SYNOPSIS

B<ble_uart.pl> B<[>OPTIONSB<]> [XX:XX:XX:XX:XX:XX[,option=value ...][,...]] 

=head1 DESCRIPTION

This script connects to one or more BLE devices implementing the Nordic UART
Service (NUS), discovers the UART RX/TX characteristics, and allows simple
UART-style read/write over BLE. It is intended for use with ESP32/ESP-AT or
similar BLE UART bridges.

Input from the terminal is sent to the BLE device, and data received from the
device is printed to the terminal with a distinct prompt and color. Multiple
connections can be managed interactively.

=head1 NORDIC UART SERVICE (NUS)

This script communicates with BLE devices that implement the Nordic UART Service
(NUS), a standard BLE service for serial-style data transfer over Bluetooth Low
Energy. NUS is commonly used on Nordic Semiconductor devices (such as
nRF52/nRF53 series) and is supported by many ESP32 BLE UART firmware projects,
including ESP-AT.

NUS provides a simple way to send and receive data as if over a UART/serial
port, but using BLE characteristics for RX (write) and TX (notify). This allows
wireless, bidirectional, low-latency communication between a host (like this
script) and a BLE device.

=head2 NUS Service and Characteristics

The Nordic UART Service uses the following UUIDs:

=over 4

=item * Service UUID: C<6E400001-B5A3-F393-E0A9-E50E24DCCA9E>

=item * TX Characteristic (notify, device to client): C<6E400003-B5A3-F393-E0A9-E50E24DCCA9E>

=item * RX Characteristic (write, client to device): C<6E400002-B5A3-F393-E0A9-E50E24DCCA9E>

=back

The host writes data to the RX characteristic, and receives notifications from
the TX characteristic.

For more information, see the official Nordic documentation:

L<https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth/services/nus.html>

=head1 ARGUMENTS

The script expects one or more Bluetooth addresses of BLE devices implementing
the Nordic UART Service (NUS). Additional options like `uart_at=0` can be
specified to disable UART AT command mode.

The format is:

    XX:XX:XX:XX:XX:XX[,option=value][,...]

Where:

=over 4

=item XX:XX:XX:XX:XX:XX

The Bluetooth address of the device to connect to.

=item option=value

An optional configuration option

=over 6

=item B<uart_at=0|1>

disable UART AT command mode. If set to 0, the script will not
send `AT` commands and will not expect `OK` responses. Default is 1 (enabled).

=item B<bt_listen_addr=BDADDR_ANY|BDADDR_LOCAL|XX:XX:XX:XX:XX:XX>

local Bluetooth address to listen on (default: BDADDR_ANY).

=back

=back


=head1 OPTIONS

=head2 Command Line Options

=over 4

=item B<--help>

Show help

=item B<--man>

Show full manual

=item B<--raw>, B<-r>

Enable raw mode - disables colored output, UTF-8 formatting, and fancy prompts.
Also sets log level to NONE unless explicitly configured. Useful for scripting
or when piping output.

=back

=head2 COMMANDS

The following commands can be entered at the prompt:

=over 4

=item B</connect XX:XX:XX:XX:XX:XX[,option=value]>

Connect to a new BLE device by address, optionally key=value pairs for config

Example: 

    /connect 12:34:56:78:9A:BC
    /connect 12:34:56:78:9A:BC,uart_at=0


=item B</disconnect [XX:XX:XX:XX:XX:XX]>

Disconnect all BLE connections, or only the specified BLE device if a Bluetooth
address is given.

Example:

    /disconnect
    /disconnect 12:34:56:78:9A:BC

=item B</script E<lt>fileE<gt>>

Execute commands from the specified file, one per line, as if entered at the
prompt. Blank lines and lines starting with '#' are ignored.

Example:

    /script mycommands.txt

=item B</exit>, B</quit>

Exit the program.

=item B</debug on|off>

Enable or disable debug logging (sets loglevel to DEBUG or INFO).

=item B</logging on|off>

Enable or disable info logging (sets loglevel to INFO or NONE).

=item B</loglevel E<lt>none|info|warn|error|debugE<gt>>

Set the log level explicitly.

=item B</help>

Show this help message (list of / commands).

=item B</usage>

Show the usage.

=item B</man>

Show the manpage.

=item B</switch E<lt>XX:XX:XX:XX:XX:XXE<gt>>

Switch the active BLE device for terminal input/output to the specified
connected device.

Example:

    /switch 12:34:56:78:9A:BC

If no address is given, lists all currently connected devices.

=back


=head1 ENVIRONMENT

The following environment variables affect the behavior of this script:

=over 4

=item B<BLE_UART_DIR>

Directory for history and config files (default: ~/.ble_uart). Note that the
defaulting is done via the HOME and LOGNAME environment variables.

=item B<BLE_UART_HISTORY_FILE>

History file location (default: ~/.ble_uart_history).

=item B<BLE_UART_RAW>

Enable raw mode (default: 0). If set to 1, disables colored output, UTF-8
formatting, and fancy prompts. Also sets log level to NONE unless explicitly
configured.

=item B<BLE_UART_LOGLEVEL>

Set the log level (default: info). Can be set to debug, info, error, or none.

=item B<BLE_UART_INTERACTIVE_COLOR>

Enable colored output (default: 1).

=item B<BLE_UART_INTERACTIVE_UTF8>

Enable UTF-8 output (default: 1).

=item B<BLE_UART_INTERACTIVE_MULTILINE>

Enable multiline input (default: 1).

=item B<BLE_UART_NUS_SERVICE_UUID>

Override the Nordic UART Service UUID (default:
6E400001-B5A3-F393-E0A9-E50E24DCCA9E).

=item B<BLE_UART_NUS_RX_CHAR_UUID>

Override the NUS RX Characteristic UUID for writing data to the device
(default: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E).

=item B<BLE_UART_NUS_TX_CHAR_UUID>

Override the NUS TX Characteristic UUID for receiving notifications from the
device (default: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E).

=item B<TERM>

Terminal type, used to determine if colors are supported, if not set, "vt220"
is assumed.

=item B<COLORTERM>

This is checked for color support, if set to "truecolor" or "24bit", it will
enable true color support. If not set, "truecolor" is assumed.

=item B<MANPAGER>

Used for displaying the manpage, see the manpage of "man". This is usually
"less". If not set, "less" is used.

=back

=head1 AUTHOR

CowboyTim

=head1 LICENSE

This software is released under the Unlicense. See L<https://unlicense.org> for
details.

=cut
