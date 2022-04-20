#! /usr/bin/python3
import click

@click.command()
def hello():
    # this can be run as ./click_basics.py
    click.echo('Hello there')

@click.command()
@click.argument('name', default='guest')
def hello_arg(name):
    # can be run as ./click_basics.py ARG
    click.echo(f'Hello {name}')


@click.command()
@click.argument('name', default='guest')
@click.argument('age', type=int)
@click.option("--n", prompt="key in a random number", help="Provide your name")
def hello_typed_arg(name, age, n):
    # default type is str. now age is int
    # in terms of the option, help will be shown in ./click_basics.py --h. Then once you type in name and age, you will type in the prompt
    click.echo(f'{name} is {age + 1} years old')

@click.command()
@click.option('-n', type=int, default=1)
def dots(n):
    # in click.option you can have either -n or --n 
    click.echo('.' * n)

import os
@click.argument('mydir', envvar='MYDIR', type=click.Path(exists=True))
@click.command()
def dolist(mydir):
    # do export MYDIR=~/Documents; ./click_basics.py you will see MYDIR
    # if this env var doesn't exist, we will have to pass in an argument
    click.echo(os.listdir(mydir))

if __name__ == '__main__':
    # hello()
    # hello_arg()
    # hello_typed_arg()
    # dots()
    dolist()
