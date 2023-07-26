#!/usr/bin/env python
# This doesn't work, need to work with login

from selenium import webdriver
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium import webdriver
import os
from bs4 import BeautifulSoup as bs
import re as re
import time
import pandas as pd
from collections import deque

from pudb import set_trace

current_file_path = os.path.realpath(__file__)
PATH = os.path.dirname(current_file_path) + "/chromedriver"
op = webdriver.ChromeOptions()
op.add_argument("headless")
driver = webdriver.Chrome(options=op)

watch_list = {
    # "klairs toner": "https://www.amazon.com/Preparation-Unscented-lightweight-essential-oil-free/dp/B07B65NJLV/ref=sr_1_1_sspa?keywords=klairs%2Btoner&qid=1690344032&sprefix=klairs%2B%2Caps%2C105&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1",
    # "cosrx": "https://www.amazon.com/COSRX-Repairing-Hydrating-Secretion-Phthalates/dp/B00PBX3L7K/ref=sr_1_1_sspa?crid=1LMCAI4VDQNPJ&keywords=cosrx+snail+96&qid=1690344936&sprefix=cosrx+snail+96%2Caps%2C106&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&psc=1&smid=A2E3ILYZ70ZIBL",
    "clean it zero": "https://www.amazon.com/BANILA-CO-Original-Cleansing-Remover/dp/B07BSVJ4H8/ref=sr_1_4?crid=3QXTHKCMIBT7H&keywords=clean%2Bit%2Bzero&qid=1690345010&sprefix=clean%2Bit%2Bzero%2Caps%2C107&sr=8-4&th=1",
    # "laroche": "https://www.amazon.com/Roche-Posay-Toleriane-Hydrating-Gentle-Cleanser/dp/B01N7T7JKJ/ref=sr_1_3?keywords=laroche-posay+cleanser&qid=1690345864&sprefix=laroch%2Caps%2C112&sr=8-3"
}
# here, the driver will proceed through each step
for name, href in watch_list.items():
    driver.get(href)
    time.sleep(0.5)
    apex_desktop = driver.find_element(By.ID, "apex_desktop")
    # set_trace()
    price_tag = apex_desktop.find_element(By.XPATH, '//td[text()="Price:"]')
    enclosing_price_tag = price_tag.find_element(By.XPATH, "..")
    price = enclosing_price_tag.find_element(By.CLASS_NAME, "a-offscreen")
    print(f'{name}: {price.get_attribute("innerHTML")}')
    # a_offscreens = apex_desktop.find_elements(By.CLASS_NAME, "a-offscreen")
    # for a in a_offscreens:
    #     print(f'{name}: {a.get_attribute("innerHTML")}')
    
while True:
    time.sleep(1)