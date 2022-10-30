## JS project: 
- add a new page: ```functions.php```, look for wordpress_post
- pages are stored in MySQL database. phpmyadmin

## Plan of Attack 
- Skeleton with public facing ability(D)
- Rest API to add recipes 
    1. Server to add recipes to SQL database
        - Different Fields: (ACF)
            - Need 2 things:  "Edit page" + local page, and change appetizer/single.hpp
                - edit page will allow you to key in field values, those are saved into mysql database
            - https://www.wpbeginner.com/wp-tutorials/wordpress-custom-fields-101-tips-tricks-and-hacks/
            - Creating a User Interface for Custom Fields, using advanced-custom-fields
            - and advanced-custom-fields table
    2. Rest API
        - https://wordpress.org/plugins/acf-to-rest-api/#developers
- User addition page
- Machine Learning Part
    - Recommendation system
