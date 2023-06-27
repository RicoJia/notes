#!/usr/bin/python3

def test_files_under_same_name():
    import glob
    prefix = "python"
    matching_files = glob.glob(f"{prefix}*")
    print("matching files: ", matching_files)


def test_cpu_limit():
    """
    1. Soft constraint is from the user configurtaion, hard is from the operating system
        i.e., if violating the hard constraint, system will shutdown

    """
    import resource

    # 1. Check resource limits, and change them
    print("desc, name, soft, hard")
    for name, desc in [
        ('RLIMIT_CORE', 'core file size'),
        ('RLIMIT_CPU', 'CPU time'),
        ('RLIMIT_FSIZE', 'file size'),
        ('RLIMIT_DATA', 'heap size'),
        ('RLIMIT_STACK', 'stack size'),
        ('RLIMIT_RSS', 'resident set size'),
        ('RLIMIT_NPROC', 'number of processes'),
        ('RLIMIT_NOFILE', 'number of open files'),
        ('RLIMIT_MEMLOCK', 'lockable memory address'),
        ]:
        limit_num = getattr(resource, name)
        soft, hard = resource.getrlimit(limit_num)
        print('Maximum %-25s (%-15s) : %20s %20s' % (desc, name, soft, hard))

    # 2. check the current process's usage
    import time
    usage = resource.getrusage(resource.RUSAGE_SELF)
    for name, desc in [
        ('ru_utime', 'User time'),
        ('ru_stime', 'System time'),
        ('ru_maxrss', 'Max. Resident Set Size'),
        ('ru_ixrss', 'Shared Memory Size'),
        ('ru_idrss', 'Unshared Memory Size'),
        ('ru_isrss', 'Stack Size'),
        ('ru_inblock', 'Block inputs'),
        ('ru_oublock', 'Block outputs'),
        ]:
        print('%-25s (%-10s) = %s' % (desc, name, getattr(usage, name)))

    # 3. set resource limit:

def test_expand_user():
    import os
    # Getting env var: HOME 
    home = os.environ.get('HOME')
    print("path: ", os.path.expanduser(f"{home}/Downloads"))


def test_argparse():
    """
    1, `--kwargs` is by default a key-worded argument
        - `argparse.add("service")` will actually become a positional arg.
            e.g., jefe sdf, sdf is "service" here
    """
    
    import sys
    # print("The first arg is the file itself", sys.argv[0], "second arg: ", sys.argv[1])

    # 2. But argparse will disable sys
    import argparse
    parser = argparse.ArgumentParser()
    # required is default false. -- is optional, without -- is positional
    parser.add_argument("--bool", action="store_false")
    parser.add_argument("nums", nargs=2)
    # nargs = "*" (>0), "+" (>=1), "?" (0/1) these are regexes
    parser.add_argument("variable_nums", nargs='*')
    # by specifying type, this could fail
    parser.add_argument("--some_int", type=int)
    # + means one or more
    args = parser.parse_args()
    print("bool should have a default value", args.bool)
    print("nums should be the first 2 args", args.nums)
    print("variable_nums are the rest of the args", args.variable_nums)

if __name__=="__main__":
    # test_files_under_same_name()
    test_expand_user()