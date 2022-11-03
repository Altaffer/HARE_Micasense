#!/bin/bash
nmcli con mod eth0 ipv4.address 192.168.10.40/24
nmcli con mod eth0 ipv4.address 192.168.1.100/24
nmcli con mod eth0 ipv4.methon manual

nmcli con up eth0
