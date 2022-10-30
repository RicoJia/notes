// not sure what h1 is? but will change the text in the html file
// But we can affect HTML in javascript using Document-Object-Model
const my_heading = document.querySelector("h1");
my_heading.textContent = "Hola, RJJE";  

// When there're errors, check <F12> on your browser
// reference to img element in HTML
const my_img = document.querySelector("img");
const FACE_PALM_SRC = "https://pictshare.net/k6ifrn.gif";
const WOW_SRC = "https://pictshare.net/98l59o.gif";

// onclick is event handler, where we pass in an anonymous function.
my_img.onclick = () => {
    const my_src = my_img.getAttribute("src");
    let my_img_name = "";
    if(my_src === FACE_PALM_SRC){
        my_img_name = "wow";
        my_img.setAttribute("src", WOW_SRC);
    }else{
        my_img.setAttribute("src", FACE_PALM_SRC);
        my_img_name = "face palm";
    }
    alert('Ouchy eslab, Now displaying: ' + my_img_name);
};

// Web Storage API: Data not sent to server, but saved to browser as (key, value) 
// 1. session storage - max 5MB data can be stored within the current tab.
// So when youc come back to the page, you get that data.
// 2. Localstorage: persists when the browser is closed. 
// Can be cleared only through JS / clearing feature
function prompt_user_name() {
    const user_name = prompt("What's your name honey?");
    // without null check, if you hit cancel, you'll get NULL
    if (user_name){
        localStorage.setItem("user_name", user_name);
        my_heading.textContent = `Hola como estas ${user_name}`;
    }
}

if(localStorage.getItem("user_name")){
    alert(`hello, nice to see you again ${localStorage.getItem("user_name")}`);
}
const enter_your_name_button = document.querySelector("button");
enter_your_name_button.onclick = prompt_user_name;


/*
Part 2 - Web server. 
1. At least an HTTP server
    - static/dynamic: will / will not update stored files
    - Python3's http.server
        - cd website root
        - python3 -m http.server <PORT_NUM>
2. whois website
    - companies like registrars, and amazon (.fire), keep track of domain names
        - you need public server to host your website
    - DNS Request Process
        1. DNS request ->  Your computer has a local DNS cache. 
        -> DNS server -> IP address
*/
