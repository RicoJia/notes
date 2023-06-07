from flask import Flask, request, render_template

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def home():
    if request.method == 'POST':
        new_value = request.form.get('new_value')
        # This keeps the target 'display-value' for future replacements
        return f"<div id='display-value'>{new_value}</div>"
    return render_template('html_macro_example/htmx_example.html')