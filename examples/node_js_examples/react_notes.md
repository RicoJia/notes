========================================================================
## Basics
========================================================================
1. Motivation: component tree 2. make your own element (like paragraphs?)
    - each component has its own css and html
    - diffing: what's changed. Much more interactive
2. File Structure
    index.js -> App.JS (App, App could be an extended module, or component) -> if App() is a component, provide render()
3. [Airbnb coding guide](https://airbnb.io/javascript/react/)
    1. One component per file, called ```component.jsx```
        - `App.js` also in `components/`
    2. states:
        - For stateless components, use functions``` function foo{return HTML}; ```
        - For stateful components, use classes ``` class Component extends React.Components{} ```

========================================================================
## Video Widget
========================================================================
### Theory
- MJPEG: basically a series of jpeg images, easy to set up
- RTMP
- HLS: (HTTP Live Stream), developed by apple.
    - Break down a video into a few second segments
- WebRTC

### Implmentation
1. MJPEG videos are easier to display in React