
from games import *

game_object = TicTacToe()
#game_object = ConnectFour() #   Gomoku() AL: it takes too long ; naive approuch to check states 3^(6*7) dosn't work ;


game_object.play_game( minmax_player ,  minmax_player ) # minmax_player , random_player query_player  alpha_beta_player expect_minmax_player

#play_game(    dict(X=random_player, O=player(alphabeta_search))    , verbose=True).utility









