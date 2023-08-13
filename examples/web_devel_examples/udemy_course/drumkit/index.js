const look_up = {
    "w": "tom-1.mp3",
    "a": "tom-2.mp3",
    "s": "tom-3.mp3",
    "d": "tom-4.mp3",
    "j": "crash.mp3",
    "k": "kick-bass.mp3",
    "l": "snare.mp3",
}

function button_animation(button_letter) {
    if (button_letter in look_up){
        var active_button = document.querySelector("."+ button_letter);
        active_button.classList.add("pressed");
        setTimeout(function(){
            active_button.classList.remove("pressed"); 
        }, 100);
    }
}
function makeSound(button_letter) {
    if (button_letter in look_up){
        mp3_file = "sounds/" + look_up[button_letter];
        var audio = new Audio(mp3_file);
        audio.play();

    }
}

// NOTE: you CANNOT do pressButton(), because you are immediately invoking the function
var drums = document.querySelectorAll(".drum");
for (let drum of drums) {
    drum.addEventListener("click", function() {
        makeSound(this.innerHTML);
        button_animation(this.innerHTML);
    });
}

document.addEventListener("keypress", function(event){
    makeSound(event.key);
    // button animation
    button_animation(event.key);
});