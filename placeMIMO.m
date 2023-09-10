function [N,L] = placeMIMO(A,B,P)
%placeMIMO  Closed-loop pole assignment for MIMO systems using state feedback.
%
%   K = placeMIMO(A,B,P) computes a state-feedback matrix K such that
%   the eigenvalues of A+B*K are those specified in the vector P. The
%   vector P may contain elements with multiplicity greater than 1.
%
%   [K,L] = PLACE(A,B,P) returns the eigenvalues of A+B*K.
%   Note that this function places the poles of the closed-loop matrix up
%   to a certain accuracy. A warning message is printed if the nonzero closed-loop
%   poles are greater than 10% from the desired locations specified 
%   in P.

% S. van den Eijnden, E. Lefeber 5-3-2019

%% Input checks
m = size(A);
n = size(B);
p = size(P);

if ~(m(1)== m(2))
    error('Input matrix A must be square')
end

if ~(n(1) == m(1))
    if ~(n(2)==m(1))
        error('B must have the same number of rows as A')
    else
        B = B.';
    end
end

if ~(p(1) == m(1))
    if ~(p(2)==m(1))
        error('P must have the same number of columns as A') 
    else
        P = P.';
    end
end

if ~(rank(ctrb(A,B)) == m(1))
   error('(A,B) not controllable')
end

%% Pole placement algorithm
% For a reference see the book: Introduction to Mathematical Systems Theory 
% by J. W. Polderman and J. C Willems. Particularly, see algorithm 9.5.1, pp. 325  

% Step 1: Find K and N1 such that (A+B*N1, B*K) is controllable.
% It can be shown that a 'random' choice for (K,N1) will do.

s=rng(2); % set seed for random number generator (for consist results of function), and save previous seed
K = rand(size(B,2),1);
N1 = rand(size(B,2),size(B,1));
A1 = A + B*N1;
B1 = B*K;
% Though pair (A1,B1) should be controllable with probability 1, check if
% it is controllable, and otherwise try again
while ~(rank(ctrb(A1,B1)) == m(1))
    K = rand(size(B,2),1);
    N1 = rand(size(B,2),size(B,1));
    A1 = A + B*N1;
    B1 = B*K;
end
rng(s); % restore the settings of random generator
    
% Step 2: Put A1 = A+B*N1, B=B*K, and compute F from
% F[B1, A1*B1,..., (A1)^{n-1}*B1] = [0 0 0 ... 1]
b = [zeros(1,m(1)-1),1];
F = b/(ctrb(A1,B1));

% Step 3: Compute N2 = -F*(r(A1)).

% Here r(A1) = r0*I + r1*A1 + r2*A1^2 + ... + rn*A1^{n-1} + A^n.
% Moreover, r(x) is the desired characteristic polynomial of the
% closed-loop matrix A+B*N. Eigenvalues are the roots, i.e., 
% r(x) = (x-p1)*(x-p2)*(x-p3)*...

% Convert the eigenvalues to characteristic polynomial coefficients
r = flip(poly(P));

N2 = -r(1:end-1)*obsv(A1,F) - F*(A1^m(1));

% Step 4: Compute final feedback law N = N1+K*N2
N = N1+K*N2;

%% Calculate eigenvalues
L = eig(A+B*N);

% Check results. Start by removing 0.0 pole locations
P = sort(P);
i = find(P ~= 0);
P = P(i);
pc = sort(L);
pc = pc(i);
if max(abs(P-pc)./abs(P)) > .1
    disp('Warning: Pole locations are more than 10% in error.')
end

end
