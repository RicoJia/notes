
// this is an arrow function, aka lambda function in python. More common because
// In funcitonal programming, functions are passed as arguments
// require(MODULE_NAM) returns a function.
const prompt = require("prompt-sync")();
const ROWS = 4;
const CLNS = 3;
// these are the symbols in a row
const get_symbols = () => {
    const SYMBOLS = {
        "1": 100,
        "2": 3,
        "3": 2,
    };
    let total_len = 0;
    // NOTE: Object.entries returns a list of key value pairs (string keyed)
    // NOTE: you have to put things in a list to unpack
    let return_ls = [];
    Object.entries(SYMBOLS).forEach(([sym, cnt]) => {
        // NOTE: have to use fill to initialize the default value
        let sym_ls = new Array(cnt).fill(sym);
        // NOTE: concat requires you to catch it
        return_ls = return_ls.concat(sym_ls);
    });
    return return_ls;
}

const symbols = get_symbols();
const get_number = (msg, failure_msg, failure_predicate) => {
    while (true){
        // Note: many operators do number coercion inside (i.e., convert to numbers from string, time objects)
        // but you still need to do number coercion yourself, because "1" + "1" = "11"
        const num = parseFloat(prompt(msg));
        if (isNaN(num) || failure_predicate(num)) {
            console.log(failure_msg + num);
        }
        else{
            return num;
        }
    }
}

const reels = () => {
    // You can generate length this way...
    let reels = Array.from({length: 3}, ()=>[]);
    for (let r = 0; r < ROWS; r++) {
        // structedCopy is introduced in nodeJS 17. Using the JSON trick
        symbols_cp = JSON.parse(JSON.stringify(symbols));
        row = [];
        for (let c = 0; c < CLNS; c++) {
            random_id = Math.floor(Math.random() * symbols_cp.length);
            row.push(symbols_cp[random_id]);
            symbols_cp.splice(random_id, 1);
        }
        reels[r] = row;
    }
    return reels;
}

const main = () => {
    let deposit = get_number("Please enter a valid number for deposit: ", "Invalid number for deposit: ", failure_predicate = (num)=>{return num <= 0;});
    let bet_row = get_number("Please enter a valid number for bet row: ", "Invalid number for bet: ", 
        failure_predicate = (num)=>{return num <= 0 || num >= ROWS;});
    let r = reels();
    console.log(r);
    // array.every element 
    let all_same = r[bet_row].every((element) => {return element == r[bet_row][0]; });
    let winning = 2 * deposit * all_same;
    // Better convert it to string
    console.log("winnings: " + winning.toString());
}

main();