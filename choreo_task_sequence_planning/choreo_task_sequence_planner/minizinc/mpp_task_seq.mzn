% Assembly Sequence Encoding
include "globals.mzn";
include "as_minizinc_data_layer_1.dzn";

% Input Constants
int: n;
int: m;

set of int: N = 1..n;
set of int: M = 1..m;

array[N] of int: G_data;
array[N] of int: G = array1d(N, G_data);

array[1..n*n] of int: A_data;
array[N, N] of int: A = array2d(N, N, A_data);

array[1..n*n*m] of int: T_data;
array[N, N, M] of int: T = array3d(N, N, M, T_data);

% Decision variables
array[N] of var N: o;

% Constraints
constraint
  all_different(o);

constraint
  forall (i in 1..n)
  (
  forall(j in 1..i-1)
  (
     A[o[i], o[j]] = 1 \/ G[o[i]] = 1
  )
  );

constraint
  forall (i in 1..n)
  (
    exists(a in 1..m)
    (
      forall(j in 1..i-1)
      (
        % printing edge o[i] with eef orientation[k] and edge o[j]'s existence is collision-free
        T[o[i], o[j], a] = 1
      )
    )
  );

solve satisfy;

output ["\nSolution:\n   i o[i]\n-------------------\n"];
output [show_int(3, i) ++ show_int(3, o[i])  ++ "\n" | i in N];