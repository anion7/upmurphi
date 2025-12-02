# upmurphi

## A* search support

This fork adds an A* search based exploration mode for PDDL+ planning problems.  
Use the new `-vastar` switch (or set the main algorithm to `Explore_astar`) to run
best-first exploration guided by action costs instead of plain BFS. The mode shares
the same output pipeline as the original planner, so subsequent phases (graph build,
plan extraction, validation, etc.) keep working transparently. If you still prefer
breadth-first search, the default `-vbfs` behaviour remains unchanged.