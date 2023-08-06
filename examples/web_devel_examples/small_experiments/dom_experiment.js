// Button: Click simulation, it's an input checkbox
document.querySelector("input").click();
// note, not inner HTML
document.querySelector("blockquote").innerText = "heeelloo \n rico";
console.log("Inner HTML: " + document.querySelector("blockquote").innerHTML);

//  set href, anchor tag inside a paragraph tag
// All Properties in JS have to be in strings
document.getElementById("sample_paragraph").attributes
document.querySelector("p a").setAttribute("href", "https://www.google.com");
// - In css, ```.class```, ```#id```, ```tag```
document.querySelector(".body_text.test_paragraph").innerText="Leihou";
// class, id, and (no space),
document.querySelector("#sample_paragraph").innerText="Sample Text";
// document.querySelectorAll
document.querySelectorAll("p");

// set style by adding a css class to an element's classlist. Note, you can't use it on multiple
// elements. There's add, remove, and toggle
document.querySelector("#sample_paragraph").classList.add("backup_rico_emoji");
// THIS IS NOT RECOMMENDED, BUT YOU CAN USE IT
document.querySelector("h1").style.color = "#77bd4c";



