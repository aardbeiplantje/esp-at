#!/bin/bash

uid=$(id -u)
socat_opt=

# ipv6 UDP IN/OUT
udp_port=56765
tgt_ipv6="[fd00::cd3e:3b67:3263:fae9]"
uart_fn=/run/user/$uid/uart_${udp_port}_udp_ipv6
socat -b8 ${socat_opt} UDP6:${tgt_ipv6}:${udp_port},ipv6only=1,reuseaddr pty,link=${uart_fn},raw,unlink-close=0 &
sleep 2
stty -F ${uart_fn} 115200 cs8 -parenb -cstopb raw -echo
cat ${uart_fn} &

# upv4 UDP IN/OUT
tgt_ipv4=192.168.1.80
uart_fn=/run/user/$uid/uart_${udp_port}_udp_ipv4
socat -b8 ${socat_opt} UDP4:${tgt_ipv4}:${udp_port} pty,link=${uart_fn},raw,unlink-close=0 &
sleep 2
stty -F ${uart_fn} 115200 cs8 -parenb -cstopb raw -echo
cat ${uart_fn} &

# ipv6 IN only
udp_in_port=56965
socat ${socat_opt} UDP6-LISTEN:$udp_in_port,ipv6only=1,reuseaddr STDOUT &

# ipv4 IN only
socat ${socat_opt} UDP4-LISTEN:$udp_in_port STDOUT &

# ipv6 OUT only
udp_out_port=56865
uart_in_fn=/run/user/$uid/uart_${udp_out_port}_udp_ipv6_in
socat -b8 ${socat_opt} pty,link=${uart_in_fn},raw,unlink-close=0 UDP6-SENDTO:$tgt_ipv6:$udp_out_port,ipv6only=1 &

# ipv4 OUT only
uart_in_fn=/run/user/$uid/uart_${udp_out_port}_udp_ipv4_in
socat -b8 ${socat_opt} pty,link=${uart_in_fn},raw,unlink-close=0 UDP-SENDTO:$tgt_ipv4:$udp_out_port &

wait
