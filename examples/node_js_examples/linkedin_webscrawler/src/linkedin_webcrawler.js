// Not supporting ES6 if it's not a module
// import * as cheerio from 'cheerio';
const cheerio = require('cheerio');

async function get_html(url){
    var response = await fetch(url);
    var html = await response.text()
    return html;
}

function process_text(text){
    // replace multiple new lines with a single new line. /regex/, g means global
    text = text.replace(/\n+/g, "\n");
    text = text.replace(/<img[^>]*>\n.*\n/g, "");
    return text;
}

var user_queries = ["tofu masala", "chinese chicken rice"];
for (let user_query of user_queries) {
    var search_query_url = "https://www.allrecipes.com/search?q=" + user_query.replace(/ /g, "+");

    get_html(search_query_url)
    .then(
        html => {
            var search_ls = [];
            const $= cheerio.load(html);
            $('#card-list_1-0').find("a").each(
                function(index, element){
                    let href = $(element).attr("href");
                    search_ls.push(href)
                }
            )
            return search_ls;
        }
    )
    .then(
        search_ls => {
            // for (let url of search_ls) {
            //     console.log("search queries: ", url);
            // }
            const promises = search_ls.map(
                url => get_html(url).then(
                    html => {
                        const $= cheerio.load(html);
                        let ingredients = process_text($('ul.mntl-structured-ingredients__list').text());
                        let steps = process_text($("#recipe__steps-content_1-0").text());
                        if (ingredients && steps){
                            console.log("=============================================================");
                            console.log("search url: ", url);
                            console.log("Ingredients:", ingredients);
                            console.log("Steps:", steps);
                        }
                    }
                )
            );
            return Promise.all(promises)
        }
    )


    // var rico_url = "https://www.allrecipes.com/recipe/22848/ashleys-chocolate-chip-cookies/";

}
