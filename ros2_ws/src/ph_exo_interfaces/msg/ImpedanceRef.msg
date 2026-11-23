# Dynamic ordinary impedance v1. One atomic reference sample.
# t: same Orin host monotonic seconds; id nonzero; seq strictly increasing.
# q,dq,k,b task order [psi,beta]; tau physical motor order [104,105].
# k,b,tau must remain unchanged until deactivate/activate.
uint64 id
uint64 seq
float64 t
bool on
float64[2] q
float64[2] dq
float64[2] k
float64[2] b
float64[2] tau
