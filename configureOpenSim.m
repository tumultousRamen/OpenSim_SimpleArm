%    Windows users must ensure OpenSim's bin directory is on the operating
%    system's PATH (search for "environment" in the Windows start menu). 
%    OpenSim's installer usually does this for you.
function configureOpenSim()
function [openSimFolder] = uiGetOpenSimFolder()
    % Prompt user for OpenSim folder.
    startPath = '';
    if strcmp(getenv('OPENSIM_HOME'), '')
        if ispc
            startPath = 'C:/';
        elseif ismac
            startPath = '/Applications/';
        end
    else
        startPath = getenv('OPENSIM_HOME');
    end
    prompt = 'Select the OpenSim installation folder.';
    if ismac
        % The Mac file dialog does not show the prompt in the file dialog.
        showMessage(['Select the OpenSim installation folder ' ...
                'in the upcoming file dialog.'], '', false);
    end
    openSimFolder = uigetdir(startPath, prompt);
    if ~openSimFolder
        showMessage('You did not select a folder.', 'Error', true);
    end
    if ismac
        [~, name, ext] = fileparts(openSimFolder);
        % If openSimFolder is '/Applications/OpenSim 4.0.1' then we'll get:
        %   name: 'OpenSim 4.0'
        %   ext:  '.1'
        % So we want to combine name and ext.
        appDir = fullfile(openSimFolder, [name ext '.app']);
        if exist(appDir, 'dir')
            openSimFolder = fullfile(appDir, 'Contents', 'Resources', 'OpenSim');
        end
    end
end

try
%% Determine the OpenSim install location.
    thisFolder = fileparts(mfilename('fullpath'));
    % Number of directories between this file and the OpenSim install root.
    numDirs = length(strfind('../../../', '..'));
    % Back up numDir times from this folder.
    openSimFolder = thisFolder;
    for i = 1:numDirs
        % This function returns all but the leaf element of the provided path.
        openSimFolder = fileparts(openSimFolder);
    end
    % For this to be an OpenSim installation, there must be a buildinfo file.
    buildInfoFile = fullfile(openSimFolder, 'sdk', ...
                'OpenSim_buildinfo.txt');
    if ~exist(buildInfoFile)
        % The script is not located in the expected location within the OpenSim
        % installation; ask the user to choose an install directory.
        correctFolder = false;
        while ~correctFolder
            openSimFolder = uiGetOpenSimFolder();
            % Check if the user selected a valid folder.
            buildInfoFile = fullfile(openSimFolder, ...
                    'sdk', 'OpenSim_buildinfo.txt');
            if exist(buildInfoFile)
                correctFolder = true;
            else
                showMessage(['The folder you selected is not an OpenSim ' ...
                    'installation.'], 'Error', false);
            end
        end
    end

%% Check if Matlab and OpenSim are compatible (64 vs 32 bit)
    checkSystemInfo(buildInfoFile);

%% Edit the Java class path (need full path for print)
    toolboxLocal = fullfile(matlabroot, 'toolbox', 'local');
    % Create the string names used
    OpenSimJarPath =  fullfile(openSimFolder, ...
            'sdk/Java', 'org-opensim-modeling.jar');
    classFileTool = fullfile(toolboxLocal, 'classpath.txt');
    % Keep track of if (1) we detected existing OpenSim
    % and (2) if we had issues removing existing OpenSim entries.
    rmPrev = [false false];
    % The verLessThan function was introduced in MATLAB 2007a.
    matlabOlderThan2012b = verLessThan('matlab', '8.0');
    if matlabOlderThan2012b 
        % The prefdir mechanism doesn't exist yet; only check and use the
        % pre-2012b mechanism.
        % Must use single | to avoid short-circuit behavior of ||.
        rmPrev = rmPrev | editPathTxtFile(classFileTool, OpenSimJarPath);
    else
        % Previous versions of this script only used the pre-2012b mechanism,
        % so even if the user has 2012b or later, we must check the pre-2012b
        % mechanism. But we won't try to add the jar path to the file in the
        % toolbox directory.
        rmPrev = rmPrev | editPathTxtFile(classFileTool, '');
        % We will only add the jar path using the prefdir mechanism.
        classFilePref = fullfile(prefdir, 'javaclasspath.txt');
        rmPrev = rmPrev | editPathTxtFile(classFilePref, OpenSimJarPath);
    end

%% Edit the Java library path.
    % Create the string names used
    OpenSimLibPath  = fullfile(openSimFolder, 'sdk/lib');
    libraryFileTool = fullfile(toolboxLocal, 'librarypath.txt');
    if matlabOlderThan2012b
        rmPrev = rmPrev | editPathTxtFile(libraryFileTool, OpenSimLibPath);
    else
        rmPrev = rmPrev | editPathTxtFile(libraryFileTool, '');
        libraryFilePref = fullfile(prefdir, 'javalibrarypath.txt');
        rmPrev = rmPrev | editPathTxtFile(libraryFilePref, OpenSimLibPath);
    end

%% Edit MATLAB path
    cleanMatlabSearchPath();
    utilitiesPath = fullfile(thisFolder, 'Utilities');
    if exist(utilitiesPath, 'dir')
        addpath(utilitiesPath);
        fprintf('-- Added %s to the MATLAB path.\n\n', utilitiesPath);
        status = savepath;
        if status ~= 0 % Status is 0 for success (see doc savepath). 
            fprintf(['-- Could not save changes to the MATLAB path. ' ...
            'Therefore, you will not have access to OpenSim-related '...
            'MATLAB utilities like osimTableToStruct(). ' ...
            'Restart MATLAB as administrator (or with sudo, on UNIX) '...
            'and re-run configureOpenSim.m.'])
        end
    else
        fprintf(['-- Could not find Utilities folder; ' ...
                'not adding to MATLAB path.\n\n']);
    end

%% Display final message.
    % We say 'API' because for Mac, the openSimFolder is an internal folder
    % inside the .app.
    msg = ['Paths have been successfully updated for the OpenSim API '...
            'installed at ' openSimFolder '.'];
    if rmPrev(1) && rmPrev(2) 
        % We detected previous OpenSim entries and couldn't remove them.
        msg = [msg ' However, we were unable to fully remove existing '...
                'installations of OpenSim from MATLAB. '...
                'Either remove them manually (see Command Window for '...
                'details), or restart ' ...
                'MATLAB as administrator (or with sudo, on UNIX) ' ...
                ' and re-run configureOpenSim.m.'];
    end
    if ispc && rmPrev(1) 
        msg = [msg ' Make sure your Windows PATH (search for ' ...
                   '"environment" in the Windows start menu) ' ...
                   'contains ' OpenSimLibPath '.'];
        % If the user does not have a pre-existing OpenSim installation,
        % we assume the OpenSim installer successfully edited the PATH;
        % no need for a message.
    end
    msg = [msg ' To complete setup, restart MATLAB. To test your set up, ' ...
           'type: `model = org.opensim.modeling.Model();`'];
    showMessage(msg, 'OpenSim successfully added', false);
catch ME
    % Introduced in version 2007b (7.4).
    throwAsCaller(ME); % Avoid a stacktrace.
end % try...catch
end % function

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function rmPrev = editPathTxtFile(txtname, spath)
    % Edits Java path .txt files. Deletes old entries and adds the new path
    % spath if spath is not empty.
    % return value:
    % (1): have we detected existing OpenSim entries?
    % (2): did we have trouble removing existing OpenSim entries.
    rmPrev = [false false];
    
    % If the file does not exist then there are no old entries to delete; if
    % we are not adding an entry, then there is nothing to do.
    if ~exist(txtname, 'file') && isempty(spath)
        return
    end

    % If the file exists, then check if it has existing OpenSim entries.
    entries = cell(0);
    if exist(txtname, 'file')
        fileIDread = fopen(txtname, 'r');
        if fileIDread == -1
            if isempty(spath)
                % This branch is unlikely.
                fprintf(['-- Unable check for existing OpenSim entries in ' ...
                         '%s. Check this file manually.\n'], txtname);
                % If spath is not empty, then we intend to write to the file,
                % and we will give a more useful message in the next few lines.
            end
        else
            C = textscan(fileIDread,'%s', 'delimiter','\n');
            [entries, rmPrev(1)] = searchForOpenSimString(C, txtname);
        end
        fclose(fileIDread);
    end

    % If we detected previous OpenSim entries or if want to add an entry,
    % then we need to be able to write to the file.
    if rmPrev(1) || ~isempty(spath)
        fileIDwrite = fopen(txtname, 'w');
        if fileIDwrite == -1
            if isempty(spath)
                % If we weren't trying to add an entry, then we don't need to
                % generate an error; just notify the user.
                fprintf(['-- Unable to remove existing OpenSim entries in ' ...
                        '%s.\n'], txtname);
                rmPrev(2) = true;
                return;
            else
                showMessage(['Unable to add OpenSim to ' txtname '. ' ...
                    'Restart MATLAB as administrator (or with sudo, ' ...
                    'on UNIX) and re-run configureOpenSim.m.'], ...
                        'Error', true);
            end
        end

        % Add the spath entry.
        entries = [entries, {spath}];
        for i = 1:length(entries)
            fprintf(fileIDwrite, '%s\n', entries{i});
        end
        fclose(fileIDwrite);
        if ispc
            msg = ['-- Added ' strrep(spath, '\', '\\') ' to ' ...
                    strrep(txtname, '\', '\\')];
        else
            msg = ['-- Added ' spath ' to ' txtname];
        end
        if rmPrev(1)
            msg = [msg ' and commented out pre-existing OpenSim entries'];
        end
        msg = [msg '.\n\n'];
        fprintf(msg);
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [newC, foundExisting] = searchForOpenSimString(C, txtname)
    % Return a new cell array with all the same entries as C, except that
    % entries containing OpenSim are commented out.
    timeStr = datestr(now, 30);
    newC = cell(0);
    foundExisting = false;
    [Cnrows, Cncols] = cellfun(@size, C, 'UniformOutput', false);
    for i = 1:Cnrows{1}
        if isempty(strfind(lower(C{1}{i}), 'opensim')) || ...
                any(regexpi(C{1}{i}, '^\s*#') == 1)
            % The line does not contain 'opensim' or is commented out ('#').
            newC = [newC, C{1}{i, 1}];
        else
            commentedOut = ['# removed by configureOpenSim.m ' ...
                     timeStr ' ' C{1}{i, 1}];
            foundExisting = true;
            newC = [newC, {commentedOut}];
            fprintf('-- Detected existing %s in %s.\n', C{1}{i, 1}, txtname);
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function checkSystemInfo(buildInfoFile)
    % check to see if the buildinfo.txt file is present. This file will give
    % us install type info that determines compatability between MATLAB and
    % opensim (both 64 bit?)
    fileID = fopen(buildInfoFile);
    OpenSimInstallInfo = textscan(fileID,'%s');
    fclose(fileID);
    platformID=char(OpenSimInstallInfo{1,1}{end,1});
    OpenSimIs64bit = not(isempty(strfind(platformID, '64')));
    MatlabIs64bit = not(isempty(strfind(mexext, '64')));
    if MatlabIs64bit && not(OpenSimIs64bit)
        showMessage(['Matlab is 64 bit. OpenSim is 32 bit. ' ...
                'Obtain 64 bit OpenSim.'], 'Error', true);
    elseif not(MatlabIs64bit) && OpenSimIs64bit
        showMessage(['Matlab is 32 bit. OpenSim is 64 bit. ' ...
                'Obtain 32 bit OpenSim.'], 'Error', true);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cleanMatlabSearchPath()
    % goes through the matlab search path and removes any folders that have
    % the strings 'OpenSim' and 'bin' in them. This is to clean out older bin
    % folders.
    
    % get the matlab path    
    matlabPath         = path;
    % matlab path is just 1 long string, so index location of ';' or ':'
    if ispc
        pathSep = ';';
    else
        pathSep = ':';
    end
    matLabFoldersIndex = strfind(matlabPath, pathSep);
    matLabFoldersIndex = [0 matLabFoldersIndex];
    % How many folders?
    nFoldersInPath     = length(matLabFoldersIndex); 

    attemptedBackup = false;
    timeStr = datestr(now, 30);
    
    % for each folder
    for i = 1:nFoldersInPath-1
        % get the start end end index for each folder name   
        startString = matLabFoldersIndex(i);
        finishString = matLabFoldersIndex(i+1);
        % ouput the folder string
        folderName = matlabPath(startString+1 : finishString-1);
        
        % check to see if the folder string contains 'OpenSim'
        if ~isempty(strfind(lower(folderName), 'opensim'))
            if ~attemptedBackup
                backup = fullfile(prefdir, ['pathdef_backup_' timeStr '.m']);
                status = savepath(backup);
                if status == 0 % Status is 0 for success (see doc savepath).
                    fprintf('-- Saved a backup MATLAB path to %s.\n', backup);
                else
                    fprintf(['-- Attempted to save a backup MATLAB path ' ...
                            'but failed.\n']);
                end
                attemptedBackup = true;
            end
            rmpath(folderName);
            fprintf('-- Removing pre-existing %s from MATLAB path.\n', ...
                    folderName);
        end     
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function showMessage(msg, title, isError)
    % If the user started MATLAB without a display, then show messages on the
    % command line.
    % http://stackoverflow.com/questions/6754430/determine-if-matlab-has-a-display-available
    if usejava('jvm') && ~feature('ShowFigureWindows')
        if isError
            error([title ': ' msg]);
        else
            disp(['-- ' title ': ' msg]);
        end
    else
        h = msgbox(msg, title);
        uiwait(h);
        if isError
            error([title ': ' msg]);
        end
    end
end

