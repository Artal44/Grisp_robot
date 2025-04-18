-module(numerl).
%-on_load(init/0).
-export([ eval/1, eye/1, zeros/2, equals/2, add/2, sub/2,mult/2, divide/2, matrix/1, rnd_matrix/1, get/3, at/2, mtfli/1, mtfl/1, row/2, col/2, transpose/1, inv/1, nrm2/1, vec_dot/2, dot/2, init/0]).

%Matrices are represented as such:
%-record(matrix, {n_rows, n_cols, bin}).
%% @doc
%% Initializes the NIF (Native Implemented Function) module.
%% This function loads the NIF library associated with the current module.
%% The `atom_to_list(?MODULE)` converts the module name to a string,
%% which is then used to load the corresponding NIF library.
%% 
%% @spec init() -> ok

init()->
    ok  = erlang:load_nif(atom_to_list(?MODULE), 0).

%%Creates a random matrix.
%% @doc
%% Generates a random NxN matrix with elements ranging from 1 to 20.
%% @spec rnd_matrix(integer()) -> matrix()
rnd_matrix(N) ->
    L = [[rand:uniform(20) || _ <- lists:seq(1, N)] || _ <- lists:seq(1, N)],
    matrix(L).

% @doc
% Evaluates a list of expressions [LeftOperand, Operator, RightOperand | Tail].
% Applies the operator to the left and right operands using a dynamically created function.
% Recursively evaluates the result with the remaining tail of the list.
% Returns the final result if the list contains only one element.
%
% @param ExprList A list of expressions.
% @return The result of evaluating the entire list.
eval([L,O,R|T])->
    % Create a fun (anonymous function) that references the function numerl:O/2
    F = fun numerl:O/2,
    eval([F(L,R) |T]);
eval([Res])->
    Res.

%% The next functions are placeholders in case NIF is not loaded and will return `nif_not_loaded`.

%%Creates a matrix.
%List: List of doubles, of length N.
%Return: a matrix of dimension MxN, containing the data.
matrix(_) ->
    nif_not_loaded.

%% Retrieves the Nth element from the given matrix.
%% @spec at(matrix(), integer()) -> any()
at(_Matrix,_Nth)->
    nif_not_loaded.

%%Returns the matrix as a flattened list of ints.
mtfli(_mtrix)->
    nif_not_loaded.

%%Returns the matrix as a flattened list of doubles.
mtfl(_mtrix)->
    nif_not_loaded.

%%Returns a value from a matrix.
get(_,_,_) ->
    nif_not_loaded.

%%Returns requested row. 
row(_,_) ->
    nif_not_loaded.


%%Returns requested col.
col(_,_) ->
    nif_not_loaded.


%%Equality test between matrixes.
equals(_, _) ->
    nif_not_loaded.


%%Addition of matrix.
add(_, _) ->
    nif_not_loaded.


%%Subtraction of matrix.
sub(_, _) ->
    nif_not_loaded.


%% @doc
%% Multiplies matrix A by B. If B is a number, it calls the function '*_num'(A, B).
%% If B is a matrix, it calls the function '*_matrix'(A, B).
%% @spec mult(matrix() | number(), matrix() | number()) -> matrix().
%% Matrix multiplication.
mult(A,B) when is_number(B) -> '*_num'(A,B);
mult(A,B) -> '*_matrix'(A,B).

'*_num'(_,_)->
    nif_not_loaded.

'*_matrix'(_, _)->
    nif_not_loaded.

%Matrix division by a number
divide(_,_)->
    nif_not_loaded.

%% build a null matrix of size NxM
zeros(_, _) ->
    nif_not_loaded.

%%Returns an Identity matrix NxN.
eye(_)->
    nif_not_loaded.

%Returns the transpose of the given square matrix.
transpose(_)->
    nif_not_loaded.

%Returns the inverse of asked square matrix.
inv(_)->
    nif_not_loaded.


%------CBLAS--------

%nrm2
%Calculates the squared root of the sum of the squared contents.
nrm2(_)->
    nif_not_loaded.

% : dot product of two vectors
% Arguments: vector x, vector y.
%   x and y are matrices
% Returns the dot product of all the coordinates of X,Y.
vec_dot(_, _)->
    nif_not_loaded.

% dgemm: A dot B 
% Arguments: Matrix A, Matrix B.
%   alpha, beta: numbers (float or ints) used as doubles.
%   A,B,C: matrices.
% Returns the matrice resulting of the operations alpha * A * B + beta * C.
dot(_,_)->
    nif_not_loaded.
