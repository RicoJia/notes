"""
1. To use flask: 
    export FLASK_APP=flask_beginner.py
    export FLASK_DEBUG=1
    flask run
"""
from flask import Flask, render_template, url_for
# __name__ will be __main__ or name of the module (if being run as submodule)
app = Flask(__name__)

@app.route('/')
def hello():
    return "hello world! Server name: " + __name__

# This is another end point
@app.route('/pens')
def hello_piyixia():
    return '<h1>SLABBY</h1>\
        <img src="https://cdn.britannica.com/70/192570-138-848FB7B3/penguin-species-places-Galapagos-Antarctica.jpg?w=800&h=450&c=crop"> '

# This is my first dynamic page
# a static webpage is controlled directly by HTML, no compilation on  the server
# render_template() will find in './templates' folder using url_for()
template_experiment_count = 0
NAMES = ("Rico", "Justine")
# Side note: leave a trailing slash as a convention. 
@app.route('/temp_vanilla/')
def template_experiment():
    global NAMES, template_experiment_count
    name = NAMES[template_experiment_count]
    template_experiment_count += 1
    template_experiment_count = template_experiment_count % len(NAMES)
    return render_template('template_experiment.html', username=name)


@app.route("/temp/")
def template_parsing():
    kwargs_for_template = {
        "food_prefix": "pork", 
        "food_suffix": "dumplings",
        "num_a": 6,
        "num_b": 9,
    }
    # This is just to test jinja2. Jinja2 was written by Armin Ronacher
    from jinja2 import Template
    t = Template("Herrrooo, {{name}}")
    str1 = t.render(name="rico")
    str2 = t.render(name="JE")
    print("Jinja 2 template can be used for string substitution easily")
    print(f"{str1}")
    print(f"{str2}")
    # after template substitution, another string is returned
    return render_template("template_experiment.html", **kwargs_for_template)
    

# Test URL with variables
@app.route('/user/<username>')
def user_name(username):
    return f"Hola {username}!"

# Test url_for
@app.route('/test_url_for')
def test_url_for():
    # Test above url 
    # see relative URL, i.e., /user/rjia
    # url_for(<endpoint-name>, <param-name>)
    return f'url_for rjia: {url_for("user_name", username="rjia")}'

if __name__ == '__main__':
    # 1. This runs on your computer's network
    # app.run(host='127.0.0.1', port=8080)
    # 2. This runs on your local network. Your address is the one in wlp9s0
    app.run(host='0.0.0.0', port=8080)

    # 3. You can also generate a custom template: https://zhuanlan.zhihu.com/p/75941916
    # Run with python -m http.server PORT_NUM
    
    # 4. HTML is the main page. Static files: they do not need to change. 
    # E.g, CSS is the layout, JS is how you interact with the page. fonts is fonts

    # Use SVG images, which uses the same HTML languages. They can also be rendered in browsers