from flask import render_template, Flask, url_for
from dataclasses import dataclass
app = Flask(__name__)

@dataclass
class Item:
    img_name: str = "",
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
    return render_template('index.html', **kwargs)

if __name__ == '__main__':
    app.run(host = "0.0.0.0", port = 5000)