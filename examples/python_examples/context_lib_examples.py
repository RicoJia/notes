import contextlib
def test_session_use():
    """
    connections to database, etc. could be expensive. "a database pool"
        - context mgmr requires you to have exactly one yield
        - all code before is the setup code
        - all code afterwards is the teardown code
    """
    class RequestContext:
        @contextlib.contextmanager
        def session(self):
            if self.s = None:
                self.s = {"haha": "haha"}
            yield self.s

    @contextlib.contextmanager
    def _reuse_request_context(self):
        with self.request_context.session():
            yield