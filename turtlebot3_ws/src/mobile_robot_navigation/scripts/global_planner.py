def heuristic():
    pass 
    
def a_star ( start , goal , grid ) :
    open_set = PriorityQueue ()
    open_set . put ((0 , start ) )
    came_from = {}
    g_score = { start : 0}
    while not open_set . empty () :
        current = open_set . get () [1]
        if current == goal :
            return reconstruct_path ( came_from , current )
        for neighbor in get_neighbors ( current , grid ) :
        tentative_g_score = g_score [ current ] + cost ( current , neighbor )
            if tentative_g_score < g_score . get ( neighbor , float ( ’ inf ’) ) :
        came_from [ neighbor ] = current
        g_score [ neighbor ] = tentative_g_score
        f_score = tentative_g_score + heuristic ( neighbor , goal )
        open_set . put (( f_score , neighbor ) )
    return None