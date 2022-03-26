#!/bin/bash

echo "-------- test1: just lol, should succeed"
./add_user_2.sh lol
./delete_local_user.sh lol

echo "-------- test2: d,r,a lol, should succeed"
./add_user_2.sh lol
./delete_local_user.sh -dra lol

echo "-------- test3: dra, lolol, should not succeed"
./delete_local_user.sh -dra lolol

rip /archive
echo "------------ Archives has been romoved"
