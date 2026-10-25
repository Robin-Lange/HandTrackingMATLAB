classdef TestAffineFit < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addTestStubsToPath(~)
            addpath(fullfile(fileparts(mfilename('fullpath')), 'stubs'));
        end
    end

    methods (TestClassTeardown)
        function removeTestStubsFromPath(~)
            rmpath(fullfile(fileparts(mfilename('fullpath')), 'stubs'));
        end
    end

    methods (Test)
        function rejectsInputWithoutThreeColumns(testCase)
            f = @() affine_fit(rand(5, 2));
            verifyThrowsMessage(testCase, f, ...
                'Input matrix X must have 3 columns for 3D plane fitting.');
        end

        function rejectsInputWithTooFewRows(testCase)
            f = @() affine_fit(rand(2, 3));
            verifyThrowsMessage(testCase, f, ...
                'Input matrix X must have at least 3 points (rows) to define a plane.');
        end

        function fitsPlaneAndReturnsFiniteOutputs(testCase)
            [xGrid, yGrid] = meshgrid(linspace(-0.2, 0.2, 5), linspace(-0.2, 0.2, 5));
            zGrid = 0.15 + 0.10 * xGrid - 0.05 * yGrid;
            X = [-0.4 + 0.01 * xGrid(:), yGrid(:), zGrid(:)];

            [T, p, xdir, ydir, yaw] = affine_fit(X);

            testCase.verifySize(T, [3, 3]);
            testCase.verifySize(p, [1, 3]);
            testCase.verifyEqual(p, mean(X, 1), AbsTol=1e-12);

            testCase.verifyGreaterThan(T(1, 1), 0);
            testCase.verifyEqual(norm(T(:, 1)), 1, AbsTol=1e-12);
            testCase.verifyTrue(all(isfinite([xdir, ydir, yaw])));
        end
    end
end

function verifyThrowsMessage(testCase, fcnHandle, expectedMessage)
    didThrow = false;
    try
        fcnHandle();
    catch exception
        didThrow = true;
        testCase.verifyEqual(exception.message, expectedMessage);
    end

    testCase.verifyTrue(didThrow, ...
        'Expected function to throw an error, but no error was thrown.');
end
