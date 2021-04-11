import pickle
import sys
import numpy as np
from collections import defaultdict


def get_expected( x ):
    expected = list()
    sess_ids = np.array( [ k[0] for k in x['history'] ] )
    for i in range( len( x['results'] ) ):
        idx = np.where( sess_ids==i )[0][0]
        expected.append( x['history'][idx][2] )
    return expected


if __name__=='__main__':
    all_x = defaultdict(list)

    correct    = list()
    incorrect  = list()
    unfinished = list()
    for index in range(8):
        path  = '../results/sim_GT_%s.pickle'%(index+1)

        with open( path, 'rb' ) as f:
            x = pickle.load( f )

        # Concatenate x from all folds
        all_x['expected'] += get_expected( x )
        all_x['results']  += x[ 'results' ]
        all_x['times']    += x[ 'times' ]

        correct.append(     sum( [ a == 'correct'    for a in x['results'] ] ) )
        incorrect.append(   sum( [ a == 'incorrect'  for a in x['results'] ] ) )
        unfinished.append(  sum( [ a == 'unfinished' for a in x['results'] ] ) )

    total = np.sum(correct) + np.sum(incorrect) + np.sum(unfinished)
    print( 'Correct:',    np.sum(correct)    / total )
    print( 'Incorrect:',  np.sum(incorrect)  / total )
    print( 'Unfinished:', np.sum(unfinished) / total )

    # For bio-inspired models, the time must be summed to 4 seconds
    t_correct =   [ all_x['times'][i] for i in range(len(all_x['results'])) if ( all_x['results'][i]=='correct' and
    print( 'T_correct',   np.mean(t_correct),   np.std(t_correct) )
