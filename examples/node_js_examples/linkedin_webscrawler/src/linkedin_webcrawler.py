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

current_file_path = os.path.realpath(__file__)
PATH = os.path.dirname(current_file_path) + "/chromedriver"
#TODO Remember to remove
print(f'Rico: {PATH}')
USERNAME = "ruotongjia2020@u.northwestern.edu"
PASSWORD = "Jtzy1012*"
print(PATH)
print(USERNAME)
print(PASSWORD)

driver = webdriver.Chrome()

driver.get("https://www.linkedin.com/uas/login")
time.sleep(3)
email=driver.find_element(By.CLASS_NAME, "username")
email.send_keys(USERNAME)
password=driver.find_element(By.CLASS_NAME, "password")
password.send_keys(PASSWORD)
time.sleep(3)
password.send_keys(Keys.RETURN)