import contextlib
def test_session_use():
    """
    connections to database, etc. could be expensive. "a database pool"
        - context mgmr requires you to have exactly one yield
        - all code before is the setup code
        - all code afterwards is the teardown code
    Use method scope takes in a method name, then wrap the function under context method, which should be in its own class
    """
    import functools
    def use_method_scope(context_method_name):
        def decorator(func):
            functools.wraps(func)
            def wrapper(*args, **kwargs):
                with context_method_name():
                    return func(*args, **kwargs)
            return wrapper
        return decorator
    class RequestContext:
        def __init__(self):
            self.s = None
        @contextlib.contextmanager
        def session(self):
            if self.s is None:
                self.s = {"haha": "haha"}
            yield self.s

    class Foo:
        def __init__(self) -> None:
            self.request_context = RequestContext()
        @contextlib.contextmanager
        def _reuse_request_context(self):
            with self.request_context.session():
                yield
        @use_method_scope("_reuse_request_context")
        def _fetch_cars(self) -> bool:
            return True