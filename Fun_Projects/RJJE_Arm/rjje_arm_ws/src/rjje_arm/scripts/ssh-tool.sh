#!/bin/bash

IP=$(sudo arp-scan -l | grep Raspberry | cut -f 1)
ssh -YC "pi@${IP}"

