from flask import render_template, Flask, url_for, request
import datetime
from dataclasses import dataclass
from pymongo import MongoClient

app = Flask(__name__)

mongo_client = MongoClient("mongodb+srv://rico_jia:Jtzy1012@rico-jia-cluster.gmi5qgo.mongodb.net/test")
app.db = mongo_client.rico_recipes

@dataclass
class Item:
    img_name: str = "",
    # name of the image file in imgs/. We cannot call url_for outside of an app function
    name: str = "",
    @property
    def alt(self):
        return self.name

# You have the default static folder path
test_img_name = "air-fryer-frozen-dumplings.svg"
items = []
index_init = False
# items = [
#     Item(test_img_name, "Dumpling"),
#     Item(test_img_name, "Noodle"),
#     Item(test_img_name, "Apple Pie"),
#     Item(test_img_name, "Pho")
# ]

@app.route('/')
def index():
    global index_init
    if not index_init:
        for recipe in app.db.rico_recipes.find({}):
            items.append(Item(test_img_name, recipe["name"]))
        index_init = True
        
    kwargs = {
        "items": items,
        "page": "index"
    }
    return render_template('index.html', **kwargs)

@app.route('/recipes/', methods=['GET', 'POST'])
def recipes():
    kwargs = {
        "items": items,
        "page": "recipes"
    }
    # Tricky: for text area, it's name under textarea name="recipe"
    name = request.form.get("recipe_name")
    recipe = request.form.get("recipe")
    timestamp = datetime.datetime.today().strftime("%Y-%m-%d-%H:%M:%S")
    # items.append(Item(test_img_name, name))
    # The new item is appended.
    # Need to match the document name
    app.db.rico_recipes.insert_one({"name": name, "recipe": recipe, "timestamp": timestamp})
    items.append(Item(test_img_name, name))
    return render_template('index.html', **kwargs)

if __name__ == '__main__':
    app.run(host = "0.0.0.0", port = 5000)