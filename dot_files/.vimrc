set compatible
syntax on
filetype indent plugin on
set nobackup
set ma	" so you can use buffer to modify disk
if has("autocmd")
    autocmd FileType python set complete+=k/home/ricojia/.vim/pydiction-0.5/pydiction isk+=.,(
endif 
set exrc
set secure
inoremap <C-S-P> <C-X><C-F>

"--------------------------------------Vim-Plug---------
" install neovim thingys
" https://www.linode.com/docs/guides/how-to-install-neovim-and-plugins-with-vim-plug/
call plug#begin()

" Preview window
Plug 'skywind3000/vim-preview'

" autosave
Plug '907th/vim-auto-save'

" finds the folder with .git as the root directory
" Plug 'airblade/vim-rooter'

" gruvbox
" Plug 'morhetz/gruvbox'

" T-Comment
Plug 'tomtom/tcomment_vim'

" vim float
Plug 'voldikss/vim-floaterm'

" undo tree
Plug 'mbbill/undotree'

"vim-instant-markdown
Plug 'instant-markdown/vim-instant-markdown', {'for': 'markdown'}

" fzf
Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
Plug 'junegunn/fzf.vim'

" vim-visual-star-search
Plug 'bronson/vim-visual-star-search'

"airline, for status bar
Plug 'vim-airline/vim-airline'

"Allows plugin menu pop-ups
Plug 'vim-scripts/AutoComplPop'
call plug#end()
"
"--------------------------------------Vundle---------
set nocompatible              " be iMproved, required
filetype off                  " required

" set the runtime path to include Vundle and initialize
set rtp+=~/.vim/bundle/Vundle.vim
call vundle#begin()

" let Vundle manage Vundle, required
Plugin 'VundleVim/Vundle.vim'

" plugin on GitHub repo
Plugin 'tpope/vim-fugitive'

" Git plugin not hosted on GitHub
Plugin 'git://git.wincent.com/command-t.git'

" The sparkup vim script is in a subdirectory of this repo called vim.
" Pass the path to set the runtimepath properly.
Plugin 'rstacruz/sparkup', {'rtp': 'vim/'}

Plugin 'git@github.com:preservim/nerdtree.git'
Plugin 'jistr/vim-nerdtree-tabs'

Plugin 'git@github.com:kien/ctrlp.vim.git'
Plugin 'git@github.com:rking/ag.vim.git'

" Experimental 
Plugin 'tpope/vim-vinegar'

" Plugin 'ycm-core/YouCompleteMe', { 'do': './install.py --clang-completer' }

set signcolumn=no
hi clear SpellBad

" All of your Plugins must be added before the following line
call vundle#end()            " required
filetype plugin indent on    " required
if executable("ag")
    set grepprg=ag\ --nogroup\ --nocolor\ --column
    let g:ctrlp_user_command = 'ag %s -l --nocolor -g ""'
	let g:ctrlp_use_caching=0
endif

"-----------------------------------------General
" Editing
set backspace=indent,eol,start	
set autoread
" set hidden	"changed files will be put into buffer, and can be accessed later. These changes are not written to disk, so you can :e to other files
set number
set softtabstop=4
set shiftwidth=4 
set tabstop=4
set autoindent
set smartindent		" for c, cpp source files
" Command comlpetion
set wildmenu
set showcmd

" search
set hlsearch incsearch
set ignorecase
set smartcase

" Enabling mouse click
set mouse=a
" connecting register to the system clipboard, you still cannot use C-S-c, but
" you can use yank :)
set clipboard=unnamedplus
" set mouse&

" Toggle clipboard
" if |$mouse == 'a' 
if !empty($SSH_CLIENT) 
	set mouse=
	echom 'mouse click disabled for SSH Session'
endif

" Uptime at least affects ycm, and might affect several others. 
set updatetime=1000	

colorscheme morning

" change directory automatically
set autochdir

" OmniCompletion
set omnifunc=syntaxcomplete#Complete
let OmniCpp_NamespaceSearch   = 1
let OmniCpp_GlobalScopeSearch = 1
let OmniCpp_ShowAccess        = 1
let OmniCpp_MayCompleteDot    = 1
let OmniCpp_MayCompleteArrow  = 1
let OmniCpp_MayCompleteScope  = 1
let OmniCpp_DefaultNamespaces = ["std", "_GLIBCXX_STD"]
"-----------------------------------------General Key Remappings
" remap leader to space, we need the first line for not having space to jump
nnoremap <SPACE> <Nop>
let mapleader=" "
let maplocalleader=" "

" Search
nnoremap <Leader>ws :/\<\><Left><Left>	" search for one whole word

" enable case sensitive/insensitive searches (sr for special search)
nnoremap <Leader>r :%s///g<Left><Left><Left>
nnoremap <Leader>sr :%s/\C//g<Left><Left><Left><Left><Left>
" block replace actually operates with vim-visual-star-search
xnoremap <Leader>r :<C-u>call VisualStarSearchSet('/', 'raw')<CR> :s///g<Left><Left>
" Search in Block 
xnoremap <Leader>br :s///g<Left><Left>


" save
nmap <C-s> :w <Enter>
" we do this since we have auto save
set noswapfile	
" shift + arrow
nmap <S-Up> V
nmap <S-Down> V
" these are mapped in visual mode
vmap <S-Up> k
vmap <S-Down> j

" Editing
" paste to previous line / next line

nnoremap <leader>p o<Esc>p
nnoremap <leader><S-p> O<Esc>p

" remap backspace into regular delete
"
" nnoremap <BS> i
" remap  for line cut (the traditional dd), dd now is regular delete 
nnoremap cl dd
nnoremap dd "_dd
nnoremap <F10> :set number! <Enter>
" clear line
nnoremap <S-c>  0<S-D>

" reload vimrc
nmap <F5> :source ~/.vimrc <CR>

" Common patterns
 " insert ``` ``` by ctrl - f1
nnoremap <F3> i``````<Esc>hhi
inoremap <F3> ``````<Esc>hhi

"-----------------------------------------Rooter
" Rooter will unset autochdir 
let g:rooter_manual_only = 1
nnoremap root :Rooter
"-----------------------------------------Auto Save
let g:auto_save = 1  " enable AutoSave on Vim startup
"-----------------------------------------Status bar
" Status bar
let g:airline#extensions#tabline#enabled = 1
let g:airline#extensions#tabline#left_sep = ' '
let g:airline#extensions#tabline#left_alt_sep = '|'

"-----------------------------------------T-comment
" <C-_> <C-_> is the default comment

" -----------------------------------------floatterm
 " c-\ c-n will let you scroll like normal
" nnoremap <F12> :FloatermNew! --cwd=<root> --autoclose=2 <CR> 
let g:floaterm_keymap_toggle = '<F12>'
let g:floaterm_keymap_new    = '<F9>'
let g:floaterm_autoclose=2
tnoremap <C-n>    <C-\><C-n>
autocmd QuitPre * exec 'FloatermKill!' 
 
 "-----------------------------------------vim instant markdown
" " remove strange chars
let &t_TI = ""
let &t_TE = ""
let g:instant_markdown_slow = 0 
let g:instant_markdown_autostart =1
" let g:instant_markdown_open_to_the_world = 1
let g:instant_markdown_allow_unsafe_content = 0
"let g:instant_markdown_allow_external_content = 0
let g:instant_markdown_mathjax = 1
"let g:instant_markdown_logfile = '/tmp/instant_markdown.log'
let g:instant_markdown_autoscroll = 1 
" let g:instant_markdown_port = 8888
" let g:instant_markdown_python = 1
" let g:instant_markdown_browser = "google-chrome-stable --new-window"
 
"-----------------------------------------youcompleteme
let g:ycm_server_keep_logfiles = 1
let g:ycm_server_log_level = 'debug'
let g:ycm_global_ycm_extra_conf = "~/.vim/bundle/YouCompleteMe/.ycm_extra_conf.py"
let g:ycm_global_ycm_extra_conf = "./vim/ycm_extra_conf.py" 
let g:ycm_allow_changing_updatetime = 0

nnoremap goto :YcmCompleter GoTo<cr>
nnoremap ty :YcmCompleter GetType<cr>
nnoremap pa :YcmCompleter GetParent<cr>
nnoremap doc :YcmCompleter GetDoc<cr>
nnoremap gb <C-o> 
nnoremap <leader>gf :YcmCompleter fixit<cr>
" close/open hover window
" nmap <leader>D <plug>(YCMHover)	
let g:ycm_auto_trigger=1
" let g:ycm_autoclose_preview_window_after_completion=1
" let g:ycm_autoclose_preview_window_after_insertion=1
autocmd FileType c,cpp let b:ycm_hover = { 'command': 'GetDoc', 'GetType', 'syntax': &syntax }

"-----------------------------------------undotree
nnoremap <leader>u :UndotreeShow<cr>

"-----------------------------------------fzf search
" searches everything in the same directory
execute "set <M-f>=\ef"
nnoremap <M-f> :Rg<space>
nnoremap <leader>f :BLines<Enter>
nnoremap <C-f> :FZF<Enter>
if executable('rg')
	let g:rg_derive_root='true'
endif

" remap file
nnoremap <leader><f4> :Files<space>
" 'ctrl-t' is how to write ctrl here
let g:fzf_action = {
      \ 'ctrl-s': 'split',
      \ 'ctrl-v': 'vsplit'
  \ }
"-----------------------------------------nerdtree
"Change splitting options, similar to :vs and :sp in defaults
let NERDTreeMapOpenSplit='s'
let NERDTreeMapOpenVSplit='v'

" nerd_tree auto start with the right dir
autocmd StdinReadPre * let s:std_in=1
autocmd VimEnter * NERDTree | if argc() > 0 || exists("s:std_in") | wincmd p | endif

" Open the existing NERDTree on each new tab.
autocmd BufWinEnter * silent NERDTreeMirror

" automatically close a tab if the only remaining window is NerdTree 
autocmd bufenter * if (winnr("$") == 1 && exists("b:NERDTree") 
			\ && b:NERDTree.isTabTree()) | q | endif

" Create a new tab
let NERDTreeMapOpenInTabSilent='<c-t>'

" refresh the nerdtree directory
function! NERDTreeToggleInCurDir()
  " If NERDTree is open in the current buffer
  if (exists("t:NERDTreeBufName") && bufwinnr(t:NERDTreeBufName) != -1)
    exe ":NERDTreeClose"
  else
    if (expand("%:t") != '')
      exe ":NERDTreeFind"
    else
      exe ":NERDTreeToggle"
    endif
  endif
endfunction
nnoremap <F2> :call NERDTreeToggleInCurDir()<cr>

" -------------------------------------- Coc-Search 
" Symbol renaming.
" nmap <F3> <Plug>(coc-rename)
" nnoremap <Leader>prw: CocSearch <C-R>=expand("<cword>")<CR><CR>

" -------------------------------------- Split Window Navigation
map <C-h> <C-w>h	
map <C-j> <C-w>j	
map <C-k> <C-w>k	
map <C-l> <C-w>l	
map <C-Left> <C-w>h	
map <C-Down> <C-w>j	
map <C-Up> <C-w>k	
map <C-Right> <C-w>l	
" nnoremap <M-PageUp> <C-W>w
" nnoremap <M-PageDown> <C-W>W
"

map <PageUp><PageUp> <C-o> 
map <PageDown><PageDown> <C-i> 
" resize window
nnoremap - <C-W><lt>
nnoremap + <C-W>>

" -------------------------------------- Tab Settings
" next tabs
nmap <C-PageUp> gT 
nmap <C-PageDown> gt
" New tab
nmap <C-t> :tabnew<Enter>

" Go to last active tab
au TabLeave * let g:lasttab = tabpagenr()
nnoremap <silent> <c-n> :exe "tabn ".g:lasttab<cr>
vnoremap <silent> <c-n> :exe "tabn ".g:lasttab<cr>

" -------------------------------------- Common Mispellings
iabbrev teh the
iabbrev adn and
iabbrev waht what
iabbrev wehn when 

" -------------------------------------- Pop-ups
highlight PmenuSel ctermbg=yellow guibg=yellow
set complete=kspell,.,b,u,]
set completeopt=menuone,longest,popup
set shortmess+=c
inoremap <expr> <CR> pumvisible() ? "\<C-y>" : "\<C-g>u\<CR>"
inoremap <expr> <C-n> pumvisible() ? '<C-n>' :
  \ '<C-n><C-r>=pumvisible() ? "\<lt>Down>" : ""<CR>'

inoremap <expr> <M-,> pumvisible() ? '<C-n>' :
  \ '<C-x><C-o><C-n><C-p><C-r>=pumvisible() ? "\<lt>Down>" : ""<CR>'
  
" Navigate the complete menu items like CTRL+n / CTRL+p would.
inoremap <expr> <Down> pumvisible() ? "<C-n>" :"<Down>"
inoremap <expr> <Up> pumvisible() ? "<C-p>" : "<Up>"

" " Select the complete menu item like CTRL+y would.
" inoremap <expr> <Right> pumvisible() ? "<C-y>" : "<Right>"
inoremap <expr> <CR> pumvisible() ? "<C-y>" :"<CR>"
" " Cancel the complete menu item like CTRL+e would.
" inoremap <expr> <Left> pumvisible() ? "<C-e>" : "<Left>"
