from flask import Flask, render_template_string, request, redirect, url_for

app = Flask(__name__)

@app.route('/')
def home():
    return render_template_string(open("templates/htmx_example.html").read())

@app.route('/increment', methods=['POST'])
def increment():
    count = int(request.headers.get('count', 0))
    return str(count + 1)

@app.route('/update_text', methods=['POST'])
def update_text():
    text = request.form['text']
    return text
if __name__ == "__main__":
    app.run(debug=True)
