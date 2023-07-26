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
#TODO Remember to remove
print(f'Rico: {PATH}')
USERNAME = ""
PASSWORD = ""
print(PATH)
print(USERNAME)
print(PASSWORD)

driver = webdriver.Chrome()

# here, the driver will proceed through each step
driver.get("https://www.linkedin.com/uas/login")
email=driver.find_element(By.ID, "username")
email.send_keys(USERNAME)
password=driver.find_element(By.ID, "password")
password.send_keys(PASSWORD)
password.send_keys(Keys.RETURN)
# keywords = "machine learning"
keywords = "reinforcement learning"
driver.get(f"https://www.linkedin.com/search/results/people/?keywords={keywords}")
result_containers = driver.find_elements(By.CLASS_NAME, "entity-result__item")
# result_containers = driver.find_elements(By.CSS_SELECTOR, "entity-result__content entity-result__divider pt3 pb3 t-12 t-black--light")

# TODO: parse through multiple webpages
hrefs = set()
while not hrefs:
    filtered_keywords = ["job", "Job", "group"]
    for result_container in result_containers:
        a_tags = result_container.find_elements(By.TAG_NAME, "a")
        for a_tag in a_tags:
            href: str = a_tag.get_attribute("href")
            if href:
                if not any([k in href for k in filtered_keywords]):
                    # finding valid name, by grabbing atag and name div
                    try:
                        name_div = a_tag.find_element(By.CLASS_NAME, "visually-hidden")
                        candidate_name = name_div.text.replace("View ", "").replace("’s profile", "")
                        if candidate_name:
                            hrefs.add(frozenset([candidate_name, href]))
                    except Exception:
                        pass
print(hrefs)

def scrape_info(href: str):
    print(f'Rico: {href}')
    driver.get(href)
    while True:
        # find_element may fail based on time?
        try: 
            experience_div = driver.find_element(By.ID, "experience")
            outer_experience_div = experience_div.find_element(By.XPATH, "..")
            # pvs_list = outer_experience_div.find_element(By.CLASS_NAME, "pvs-list")
            pvs_list = outer_experience_div.find_element(By.CSS_SELECTOR, "ul:contains('pvs-list')")
            pvs_elements = outer_experience_div.find_element(By.CSS_SELECTOR, "li")
            # pvs_elements = pvs_list.find_elements(By.XPATH, ("//div[contains(@class, 'pvs-entity')]"))
            #TODO Remember to remove
            print(f'Rico: {len(pvs_elements)}')
            break
        except Exception:
            continue


scrape_info(href)
    
while True:
    time.sleep(1)