from flask import render_template, Flask, url_for, request
import datetime
from dataclasses import dataclass
app = Flask(__name__)

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
items = [
    Item(test_img_name, "Dumpling"),
    Item(test_img_name, "Noodle"),
    Item(test_img_name, "Apple Pie"),
    Item(test_img_name, "Pho")
]

@app.route('/')
def index():
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
    recipe = request.form.get("content")
    timestamp = datetime.datetime.today().strftime("%Y-%m-%d")
    items.append(Item(test_img_name, name))
    # The new item is appended.
    return render_template('index.html', **kwargs)

if __name__ == '__main__':
    app.run(host = "0.0.0.0", port = 5000)