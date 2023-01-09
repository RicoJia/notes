#!/usr/bin/env python3
import requests
import bs4

# import webbrowser
# Opening a web browser is the only thing webbrowser can do
# webbrowser.open("http://www.youtube.com")

if __name__ == '__main__':
    resp = requests.get('https://www.chinasichuanfood.com/pork-and-mushroom-stir-fry/')
    # This will raise an exception if downloading fails
    # Always do this!
    resp.raise_for_status()
    if resp.status_code == requests.codes.ok:
        print(resp.text)

    # UTF-8 is how most web pages are encoded.

    # HTML can have its strings formatted very differently.
    # So do not use regex to parse HTML, use bs4