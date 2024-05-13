import concurrent
import inspect
import threading
import traceback
from concurrent import futures
from functools import wraps

from kivy.base import EventDispatcher, ExceptionHandler, ExceptionManager
from kivy.clock import mainthread


class ThreadedException(Exception):
    """[Used as a wrapper around any exception thrown in the functions where the threaded decorator is used.

        Small hack to let exceptions be thrown but now the handler can check if exceptions were thrown in
        a thread or not. This is used to print the traceback nicely in TopSideApp main.
    ]

    Args:
        Exception ([type]): [The exception to wrap]
    """

    def __init__(self, e):
        self._exception = e
        super(ThreadedException, self).__init__(self._exception.message)

    def __str__(self):
        """[Recreate the look of the composed exception so the string doesnt read "ThreadedException: <message>"
        but instead uses the name of the underlying exception]

        Returns:
            [type]: [description]
        """
        return self._exception.__str__()

    def __repr__(self):
        """[Recreate the look of the composed exception so the string doesnt read "ThreadedException: <message>"
        but instead uses the name of the underlying exception]

        Returns:
            [type]: [description]
        """
        return self._exception.__repr__()


class ExceptionDispatcher(EventDispatcher):
    """[Used to communicate with the Kivy event system to alert the main thread that an exception has happened]

    Args:
        EventDispatcher ([type]): [description]
    """

    def __init__(self, **kwargs):
        self.register_event_type("on_dispatch_event")
        super(ExceptionDispatcher, self).__init__(**kwargs)

    def raise_exception_event(self, exception, tb):
        """[Set up exception event with the actual event and the traceback as a string]

        Args:
            exception ([Exception]): [the exception thrown in the threaded decorated function]
            tb ([str]): [The traceback to the thrown excpetion]
        """
        self.dispatch("on_dispatch_event", exception, tb)

    @mainthread
    def on_dispatch_event(self, *args):
        """[Sets the handler exception message to the traceback provided and actually raises the exception in the main
        thread. MUST be run in the main thread otherwise the ExceptionHandler will not actually catch the exception at all.]

        Raises:
            args: [exception, traceback]
        """
        # Set the tb to the
        ThreadedExceptionHandler.set_last_traceback(args[1])
        raise args[0]


class ThreadedExceptionHandler(ExceptionHandler):
    """[Manages any kivy exceptions that run in the main thread. Overwrites handle_expection
    and tells the main thread to raise the exception thrown by the event dispatcher. This kills the app and triggers
    the try catch in TopsideApp main.]

    Args:
        ExceptionHandler ([type]): [description]

    Returns:
        [type]: [description]
    """

    _traceback = None

    @staticmethod
    def set_last_traceback(tb):
        ThreadedExceptionHandler._traceback = tb

    @staticmethod
    def has_threaded_traceback():
        return ThreadedExceptionHandler._traceback is not None

    @staticmethod
    def print_tb():
        print(ThreadedExceptionHandler._traceback)

    def handle_exception(self, inst):
        """[Overwritten method that is triggered by an exception being raised in the main thread.

        If the excpetion was NOT raised in a thread we set the tracback to be None as the traceback
        will already have been printed.
        ]

        Args:
            inst ([Exception]): [description]

        Returns:
            [type]: [description]
        """
        # If we're a non threaded exception clear it so that the TopSideApp knows to print using the stacktrace
        if not isinstance(inst, ThreadedException):
            ThreadedExceptionHandler._traceback = None

        return ExceptionManager.RAISE


def _extract_function_calling_args(args):
    """[Formats the input args for the calling function to deal with the different variations of member/non-member
    functions with/without additional arguments.

    Member functions must be called func(self, args...) and non-member functions can be called func(args...). In the
    case where there are no args (this also means this function must be a non-member function) we return None.

    ]

    Args:
        function_name ([str]): [name of the function we are calling in threaded]
        args ([tuple]): [forwaded vargs given to the threaded decorator]

    Returns:
        [List]: [List of args to call. If None then call the function without any args. Else, convert
        to vargs. ]
    """
    # No args for the decorated function and is also a non-member function
    if len(args) == 0:
        return None
    else:
        return list(args)


def _run_func(fn, func_args):
    """[Correctly calls a function fn based on the given args.

    func args should be provided by a call to _extract_function_calling_args.]

    Args:
        fn (function): [Function wrapped by @threaded to call.]
        func_args ([List]): [Function args provided by *args in threaded.]
    """
    if func_args == None:
        fn()
    else:
        # If member function then self has already been pre-packaged into func_args + trailing args
        # if they exist.
        fn(*func_args)


def threaded(fn):
    """[Decorator to run the provided function in its own thread. Useful to run long tasks or asyc functions to
    prevent blocking the gui.

    All arguments will be forwarded to the functions and this decorator works with member/non-member functions as well
    as functions with and without arguments. ]

    Args:
        fn (function): [description]

    Raises:
        te: [description]

    Returns:
        [type]: [description]
    """

    @wraps(fn)
    def wrapper(*args, **kwargs):
        func_args = _extract_function_calling_args(args)
        # We need another wrapper here so we can catch the exception in the thread context and hand
        # it back to the main thread using exceptions
        def threaded_wrapper(fn):
            try:
                _run_func(fn, func_args)
            except Exception as e:
                tb = traceback.format_exc()
                te = ThreadedException(e)
                dispater = ExceptionDispatcher()
                # tell the dispatcher what the event and the traceback is so we can actually print it to screen
                dispater.raise_exception_event(te, tb)
                # Raise the exception in the thread so that the exception handler can throw it in the main thread
                raise te

        thread = threading.Thread(target=threaded_wrapper, args=(fn,), kwargs=kwargs)
        thread.start()
        return thread

    return wrapper
