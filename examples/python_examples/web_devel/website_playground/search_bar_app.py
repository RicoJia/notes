from flask import Flask, render_template, request, jsonify
# __name__ will be __main__ or name of the module (if being run as submodule)
app = Flask(__name__)

@app.route('/')
def hello():
    return render_template("search_bar.html")

@app.route('/search', methods=['GET'])
def test_search():
    # Using flask's request
    query = request.args.get("query")
    results = [query+"_test_msg", query+"_test_msg_2"]
    # Need to return json as part of DOM response for AJAX
    return jsonify(results)