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

#########################################################
# 1. group of commands using cli commands
@click.group()
def cli():
  pass
@cli.command(name='gen')
def generic():
    click.echo('Hello there')
@cli.command(name='wel')
def welcome():
    click.echo('Welcome')
def test_group_cli_command(): 
    cli()

# group of commands using click group
@click.group()
def messages():
  pass
@click.command()
def generic_cp():
    click.echo('Hello there')
@click.command()
def welcome_cp():
    click.echo('Welcome')
messages.add_command(generic_cp)
messages.add_command(welcome_cp)
def test_group_click_group():
    messages()

# add a subgroup of command to another group
# now to run this, you need ./click_basics.py messages-grp welcome-grp-cp
@click.group()
def messages_grp():
  pass
@messages_grp.command()
def welcome_grp_cp():
    click.echo('Welcome_grp_cp')
messages.add_command(messages_grp)


if __name__ == '__main__':
    # hello()
    # hello_arg()
    # hello_typed_arg()
    # dots()
    # dolist()
    # test_group_cli_command()
    test_group_click_group()
