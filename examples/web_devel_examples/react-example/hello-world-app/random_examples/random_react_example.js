/**
 * React: DOM component tree. HTML -compiler-> JS -> DOM. 
 *  - The HTML is actually JSX (js xml) 
 * In html, there's always a div called root
 *  - React is the "compiler". Without it it won't work
 *  - ReactDOM.render(JSX, where to insert it)
 *  - react strict mode is pretty good
 * img tags need alt
 */
import React from "react";
import ReactDOM from "react-dom";

const name = "RJJE";
const currentYear = new Date().getFullYear();

ReactDOM.render(
  <React.StrictMode>
    <div className="sampletext">
      <p contentEditable="true">
        Hello world, {name}, in {currentYear}
      </p>
    </div>
  </React.StrictMode>,
  document.getElementById("root")
);

/**
 * - React: we are in JS afterall, so we need to represent everything, including the style, as a JS object. Everything should be in a key-value pair, with strings
    ```javascript
    const customStyle = {fontSize: "20px", color: "red", border: "1px solid black"};
    <h1 style={{ color: "red" }}>
    <h1 style={customStyle}>
    ```
 */