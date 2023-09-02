/**
 * Jquery Example
 * Functional: 
    - Generate a random number. Push it into array 
    - 4 buttons, button w/ the right index changes color. play sound.
    - Detect Key presses. If wrong, restart
 * Front end:
    - Font
    - Button classes with colors, play sound
 */
var keypress = [];
const colors = ["black", "white"];
STATE="PLAY";

function random_color(){
    return colors[Math.floor(Math.random()*colors.length)]; 
}

/**
 * Animate flash of button, optionally with sound.
 * @param {number} button_idx - Index of the button pressed.
 * @param {boolean} with_sound - Whether to play sound
 */
function animate_flash_with_sound(button_idx, with_sound=true){
    if (button_idx != undefined){
        // Note: elements in this array returned here has order of appearance, from the HTML DOM model
        let button = $(".btn").eq(button_idx);
        let orig_color = button.css("background-color");
        // button.css("background-color", random_color());
        button.animate({backgroundColor: random_color()}, 100,
            function() {
                button.animate({backgroundColor: orig_color}, 100);
            }
        );
        if (with_sound){
            let button_name = button.attr("id");
            let a = new Audio("sounds/"+button_name+".mp3");
            a.play();
        }
    }
}

// Following convention from ECMA6
class StateMachine{
    constructor(){
        // builing button char->index map
        this.button_char_2_idx = {};
        for (let idx in $(".btn")){
            this.button_char_2_idx[$(".btn")[idx].innerText] = idx;   
        }
        this.reset();
    }

    reset(){
        this.next_keypress = null;
        this.score = 0;
        $("h1").text("Press Any Key To Start");
        $("h2").text(`Score: ${this.score}`);
        $("h2").css("color", "pink");
    }
    fail(){
        alert("Game Failed! Press Enter to restart");
    }
    /**
     * Converts keystroke to button index, animates it, checks if it is correct,
     * and sets next keypress
     * @param {char} char - Key that was pressed.
     */
    step(char){
        var button_idx=this.button_char_2_idx[char]; 
        animate_flash_with_sound(button_idx);
        if (this.next_keypress != null){
            if (button_idx != this.next_keypress){
                console.log(`Pressed ${char}, expected ${this.next_keypress}`);
                this.fail();
                this.reset();
                return;
            }
        }
        $("h1").text("Press Last Flashed Key To Score");
        $("h2").text(`Score: ${this.score}`);
        // we are proceeding to set the next keystroke
        this.score += 10;
        this.next_keypress=Math.floor(Math.random()*4);
        animate_flash_with_sound(this.next_keypress, false);
    }
}

/**
 * Callback for keypress
 * @param {event} event: keypress event
 */
function response_cb(event){
    let char = String.fromCharCode(event.which);
    sm.step(char);
}

let sm = new StateMachine();
$(document).keypress(response_cb);


/**
 * Jquery Summary:
 * - Jquery lib is pretty lightweight, pulled from CDN (content distributed Network)
 *  - Goal: make querying faster
 * - $("hi"): selecting all elements with class "hi"
 *  - $("hi").addClass("AnotherClass"): can add another CSS class
 *  - removeClass(), removeClass()
 *  - $("h1").text("Change to this new text")
 *  - $("h1").html("<em>html</em>")
 *  - $("h1").attr(src, "New_Path")
 * - Event Callbacks
 *  - $("document").keypress(
 *     (event) => {if (event.key == 'a'){...}}
 * )
 * - $("button").click(...)
 * - animation slideToggle()
 */