import os
import sys
import unittest

from mock import MagicMock, Mock, patch

import seawolf.utils.ThreadingSupport as ts


# Note: In python2.7 all tests must be called test_*
# Currently unsure how to mock the ExceptionDispatcher and ThreadedExceptionHandler classes to check they
# are called with the correct inputs
class TestThreadingSupport(unittest.TestCase):
    def test_arg_extraction_single_func(self):
        args = ()
        result = ts._extract_function_calling_args(args)
        self.assertIsNone(result)

    def test_arg_extraction_single_func_with_args(self):
        instance = MagicMock()
        args = (instance,)
        result = ts._extract_function_calling_args(args)
        self.assertTrue(isinstance(result, list))
        self.assertTrue(len(result) == 1)
        self.assertTrue(result[0] == instance)

    def test_run_args_none(self):
        func = MagicMock()

        ts._run_func(func, None)
        assert func.called

    def test_run_args_instance_no_args(self):
        class_instance = MagicMock()

        func = MagicMock()
        class_instance.mock_func = func()

        args = (class_instance,)
        ts._run_func(func, args)
        func.assert_called_with(class_instance)

    def test_run_args_instance_with_args(self):
        class_instance = MagicMock()

        func = MagicMock()
        class_instance.mock_func = func()

        args = (class_instance, 3, 4)
        ts._run_func(func, args)
        func.assert_called_with(class_instance, 3, 4)
