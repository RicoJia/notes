#! /usr/bin/python3      
import re
from typing import Pattern 

def test_simple_regex():
    # [A-Za-z0-9]+[.-_] is a group >1 char in A-Z, a-z, 0-9
    # Then followed by ., -, or _
    # Then followed by @
    # Then followed by a group, >1 char, or - 
    # Then, matches a group > 2 \.or char
    email_regex: Pattern = re.compile(r'([A-Za-z0-9]+[.-_])*[A-Za-z0-9]+@[A-Za-z0-9-]+(\.[A-Z|a-z]{2,})+')

    text = "john.doe@example.com or jane_smith123@example.co.uk"
    matches = re.findall(email_regex, text)
    print(matches)

if __name__ == "__main__":
    test_simple_regex()
