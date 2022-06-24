%%
fprintf(' * Loading hessian ... ');
%H = load('/tmp/roamfree/H.txt');
H = load('/home/davide/Code/dn/build/debug/H.txt');
H_1_sp = spconvert(H);
fprintf(' done.\n');

%%
dd = diag(H_1_sp);

[m, im] = sort(dd, 'ascend');
[M, iM] = sort(dd, 'descend');

M(1:20)

%%

MAP = []

% f = fopen('/tmp/roamfree/Hstruct.txt');
f = fopen('/tmp/shiny/roamfree/Hstruct.txt');

while ~feof(f)
    l = fgetl(f);
    
    [tokens, match] = regexp(l, 'Camera_feat(\d+)_Lw\[\d+\.\d+\]: (\d+) to (\d+)', 'tokens', 'match');
    if ~isempty(match) 
       rows = str2double(tokens{1}{2}):str2double(tokens{1}{3});
       fid = str2double(tokens{1}{1});
       
       MAP(rows+1) = fid;
    end
end