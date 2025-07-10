call plug#begin('~/.vim/plugged')  " Thư mục lưu các plugin
Plug 'preservim/nerdtree'
Plug 'vim-airline/vim-airline'
Plug 'vim-airline/vim-airline-themes'
Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
Plug 'junegunn/fzf.vim'
Plug 'tpope/vim-surround'
Plug 'altercation/vim-colors-solarized'
call plug#end()

nnoremap <C-n> :NERDTreeToggle<CR>
nnoremap <F3> :set number!<CR>
