%%
fprintf(' * Loading hessian ... ');
%H = load('/tmp/roamfree/H.txt');
H = load('/home/davide/Code/dn/build/debug/H.txt');
H_1_sp = spconvert(H);
fprintf(' done.\n');

%%
