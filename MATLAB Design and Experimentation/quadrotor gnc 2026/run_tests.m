% RUN_TESTS  Add src/ and test/ to the path and run the full test suite.
%   Run this from the project root (the folder containing src/ and test/).
%
%   Runs two kinds of tests:
%     1. matlab.unittest classes in test/ (e.g. QuaternionTest.m), via
%        runtests -- proper assertions, returns a results object.
%     2. Standalone script-style tests (e.g. gyroIntegrationDemo.m,
%        madgwickStaticTest.m) that use plain assert() and error out on
%        failure instead of returning a result object. Each one is run
%        through the local runScript() helper below, NOT called
%        directly -- see the comment on that function for why.
%
%   Add new script-style tests to the scriptTests list as you write
%   them. Adjust the addpath calls below if a script test doesn't live
%   in src/ or test/.

addpath('src');
addpath('test');

% --- 1. matlab.unittest suite -------------------------------------------
results = runtests('test');
unittestPassed = nnz([results.Passed]);
unittestFailed = nnz([results.Failed]);
unittestIncomplete = nnz([results.Incomplete]);

if any([results.Failed])
    disp(results);
end

% --- 2. Standalone script-style tests ------------------------------------
scriptTests = {
    'GyroIntegrationTest'
    'madgwickNoMagTest'
};

scriptPassed = 0;
scriptFailed = 0;
scriptFailures = {};

for i = 1:numel(scriptTests)
    name = scriptTests{i};
    fprintf('--- Running %s ---\n', name);
    try
        runScript(name);
        scriptPassed = scriptPassed + 1;
    catch err
        scriptFailed = scriptFailed + 1;
        scriptFailures{end+1} = sprintf('%s: %s', name, err.message); %#ok<AGROW>
        fprintf('FAILED: %s\n%s\n', name, err.message);
    end
end

% --- Combined summary -----------------------------------------------------
totalPassed = unittestPassed + scriptPassed;
totalFailed = unittestFailed + scriptFailed;

fprintf('\nunittest:  %d passed, %d failed, %d incomplete\n', ...
    unittestPassed, unittestFailed, unittestIncomplete);
fprintf('scripts:   %d passed, %d failed (of %d)\n', ...
    scriptPassed, scriptFailed, numel(scriptTests));
fprintf('\nTOTAL: %d passed, %d failed\n', totalPassed, totalFailed);

if ~isempty(scriptFailures)
    fprintf('\nScript test failures:\n');
    for i = 1:numel(scriptFailures)
        fprintf('  %s\n', scriptFailures{i});
    end
end

% --- Local helper ----------------------------------------------------------
function runScript(name)
% RUNSCRIPT  Run a script-style test in its OWN workspace.
%   MATLAB's run() executes a script in the CALLING workspace -- called
%   directly from the block above, gyroIntegrationDemo's/madgwickStaticTest's
%   variables (q, dt, nIterations, results, ...) would land right in this
%   script's own workspace and could silently clash with run_tests.m's own
%   variables of the same name (results, in particular, would collide
%   outright). Wrapping the call in this local function gives each script
%   test its own function-scoped workspace instead, so it can't touch
%   anything up in run_tests.m regardless of what variable names it uses
%   internally. (Local functions in script files need MATLAB R2016b+.)
    run(name);
end