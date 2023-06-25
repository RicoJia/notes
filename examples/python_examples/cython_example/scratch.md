GOAL: A GENERAL ROBOTICS SOFTWARE ENGINEER WHO CAN SETUP THE STRUCTURE OF A WELL-FUNCTIONING SYSTEM!!
- I feel that Diligent under Vivian is more vibrant, and more start-up like!
- What about we open source this project? And write blog posts about it.
- 10000h tracker: 
    - Website Building: 130h
    - Robotics Infrastructure: 2160h 
    - Machine Learning: 60h (Starting August)

## Week 22 (First wk of June)
- Personal
    - Sell the chair
- Personal Projects
    - Watch web development course. Build examples
    - Gaming Machine vs laptop?
    - Search Engine (1 wk D)
        - Better: 
            - hit, fuzzy search
            - search by ingredient
        - **HTMX** library for flask, and use a macro to dynamically save and load a page
    - Fix font
    - Web scrawler
    - Talk to Matt;
- Diligent Growth
    - Review Simulation (0.5h)
    - Debugger?
- Work:
    - Review George's Work (IP)
    - Hard Failures (IP)
    - Soft Failures
    - Emitter Failure??

- QUestions
    - how to do memory leak of cython?
    - Should You Have a Trailing Slash at the End of URLs?
    - what does absolute/ positioning mean
    - What do you think you need for your web design? https://www.trackawesomelist.com/protontypes/awesome-robotic-tooling/readme/
- READS:
    - https://zhuanlan.zhihu.com/p/126712570
    - https://news.ycombinator.com/item?id=27794248


## Week 25
### June 18
- Vision
- Current Technology
    - Reinforcement Learning.
        - Why useless
            - Still a lot of $$ flows to game
        - Robot Folding clothes, Opening Door, still seems to be far??
            - Continuous action space: 10^9. More complicated than RL applied in a game. Which has clear actions.
            - There's RL mechanism that solves door (2021)
            - 80% of AI research output is to perform one specific task.
            - Tuning, and tuning, until it solves more instances of this problem
        - AI is probably over-hyped
            - Production Reinforcement learning models?
            - does incremental learning and “lifelong learning”?
    - Computer Vision:
        - Semantic Segmentation?
- Problems
    - Data Problem? For Training Reinforcement Learning?
    - Light hill debate: https://www.youtube.com/watch?v=yReDbeY7ZMU


### June 19
- Review the rita script (20 min D)

### June 20
- George's PR 
    - Read the bash script (70 min?)
        - Logger_type?
        - String concatenation?
        - `selected_flag=${1:---h}` defaulting?
        - `[[ -n str ]]` is to check if a string is not null
        - Sharing variables across two different functions? 
        - shift? why shift 2?
        - "Usage: $0 () | -- something"
        - sed is a stream editor
    - Play with the bash script and rewrite (30 min)
- Hard Failures
    - design (5 min)
    - Compare changes
    - Implement.
    3. Hard Failure Test:
        - Create a failure in Manual Request mode.
- Debugger on flexbe?
    - pudb for debugging ros / try pdb

### June 21
- Help Anna with set trace
- Hard Failures; soft failures
    - talk to stone 
    - mark the rest
    - Smoke test?
- Another pass on George's PR
    - Review the parse script
- Talk with Ben to confirm. Talk with andrew, confirm disembark won't interfere with elevator.

- Deep Blue Academy

### June 22
- Create a new PR (D)
- Read George's PR
- Contact Stone, add the states to the state machine
- Standup: 
    - create a "watch-ticket" on the drawer issue / send back to rare for watching validating?
    - Tell em about the creation of a new ticket.
- Test hard failures
- be careful with s3-bag-indexermanual failure
- Practice script:
    - 
Add messages to distinguish soft failures (failures that lead to a manual behavior) vs hard failures (failures causing the behavior to fail)

Identify where both types of failures occur
### June 23
========================================================================
##  
========================================================================
## To explore
- 炒美股
- light curtains? (from jon)
- Moxi State

- RJJE ARM clean up (1h)
    demo.launch for move group hand service
    motion_controller.py
    send_response revamp

## Learnings

- Python
    - Be careful with id, it's a memory address. The object might have been destructed
    - expanduser so it can read from env var
        ```
        home = os.getenv("home")
        os.path.expanduser(f"{home}/Downloads/")`
        ```
    - `nargs`

- HTML:
    - http hyper text transfer protocol. Protocal to follow. Specified in HTML request, can be seen in <F12>->Network. 200 means ok, 301 means redirected
    - URL, uniform resource locator: like https:/asdfasdf.com. becomes 192.xx through DNS (domain name service) layer. Browswer now allows you to omit https because now people are more comfortable with https protocal
    - Https listens to port 443 by default;
    - browswers may disagree on how to render html. So online validator is good
    - Mark up language just puts the layout of a webpage. 
    - Theory: the brwoser, written in C/C++, builds a tree of HTML element with pointers, etc. in **memory**. This is dynamic, so Javascript can add something to this tree. The tree is called a DOM
        - javascript's `document.querySelector("id")` can work with any id on the current webpage

    - spacing: there will be two lines in the if after rendering
        ```
        <div>
            {% if true %}
                yay
            {% endif %}
        </div>
        ```
        - but with `-`. you will see this: `<div>yay</div>`
            ```
            <div>
                {%- if true -%}
                    yay
                {%- endif -%}
            </div>
            ```
        - Or, in flask: this will keep the indentation, but remove new lines
            ```
            app.jinja_env.lstrip_blocks = True
            app.jinja_env.trim_blocks = True
            ```
    - Escape. Escape generally means "skip rendering something". 
        ```
        # 1. for main.jinjia2
        # app
        return render_template("main.jinjia2, html_code="<div>hej</div>")
        # main.jinjia2 file, a pipe to escape filter
        {{ html_code | e }}

        # BUT if this is main.html, flask will AUTO-ESCAPE
        so in main.html, 
        {{ html_code }}
        If you want to actually evaluate it,
        {{ html_code | safe }}

        # In main.html, if you
        ```
    - Javascript
        ```Javascript
        let var=2;
        var += 1;
        var++; 
        if (){

        }
        while(){

        }
        ...
        //things are the same as C
        for (let i = 0; i <  5; ++i){

        }

        // you can add event here, or NOT. Javascript is very flexible
        // When a menu changes
        document.querySelector('menu').onchange = function(){
            let var = this.value; //this is document.querySelector('menu'), the element that changed.
        }
        ```
    - CSS
        - flex: flexbox

    - with flask, CSS, JS can be loaded with refresh. But with html, you have to restart the server.

- Bash: 
    ```
    a = 0
    [[ $a ]]
    ```
    - This will evaluate to True, because bash checks if this variable exists, not its value
    - nameref
        ```ref
        # This creates a local name ref to another variable
        local -n ref=another
        ```
    - Interesting use of background job to listen to 2 topics: `rostopic echo /rico & rostopic echo /george`
        

- github
    - git checkout dev -- diligent_ws/src/moxi_apps/moxi_training_pipeline/scripts/express_elevator_button_training.py


- How to add a failure state:
    1. Go to state function, 
        - entry 
        - body
        userdata.failure_category = FailureCategory.UNDOCK_FAILURE
        userdata.failure_message = "Undocking action is no longer active, and did not return a result. Something is wrong"
    2. in super.__init__: 
    output_keys=["failure_category", "failure_message"]
        
    3. If there's a container
        output_keys=[]
        - add the output keys

    - TODO: 
        - check succeed status for the two output states?
        - WHat if userdata is not available? (failure)
        - Finish the rest of the container
        - self.name



- Fix the title
