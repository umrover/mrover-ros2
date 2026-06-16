"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.activate = activate;
exports.deactivate = deactivate;
const vscode = require("vscode");
const fs_1 = require("fs");
async function callInInteractiveTerminal(command) {
    const terminal = vscode.window.createTerminal({
        name: 'mrover build',
        location: vscode.TerminalLocation.Panel,
    });
    terminal.show();
    terminal.sendText(command, false);
    terminal.sendText('; exit');
    return new Promise((resolve, reject) => {
        const disposeToken = vscode.window.onDidCloseTerminal(async (closedTerminal) => {
            if (closedTerminal === terminal) {
                disposeToken.dispose();
                if (terminal.exitStatus !== undefined) {
                    resolve(terminal.exitStatus);
                }
                else {
                    reject(new Error('Terminal exited with undefined status'));
                }
            }
        });
    });
}
async function readExecutables(directoryPath) {
    try {
        const entries = await fs_1.promises.readdir(directoryPath);
        return entries.filter(f => !f.endsWith('.py') && !f.endsWith('.sh'));
    }
    catch (err) {
        vscode.window.showErrorMessage(`Failed to read directory: ${err}`);
        return [];
    }
}
function activate(context) {
    const disposable = vscode.commands.registerCommand('file-selector.fileSelect', async (directory) => {
        const folders = vscode.workspace.workspaceFolders;
        if (!folders) {
            vscode.window.showErrorMessage('No workspace open');
            return undefined;
        }
        const workspaceRoot = folders[0].uri.fsPath;
        await callInInteractiveTerminal(`${workspaceRoot}/build.sh Debug`);
        const targetDir = `${workspaceRoot}/${directory}`;
        const executables = await readExecutables(targetDir);
        if (executables.length === 0) {
            vscode.window.showErrorMessage(`No executables found in ${targetDir}`);
            return undefined;
        }
        const options = executables.map(f => ({ label: f, description: f }));
        const selected = await vscode.window.showQuickPick(options, {
            placeHolder: 'Select an executable to debug',
            title: 'Choose an Action',
        });
        return selected?.label;
    });
    context.subscriptions.push(disposable);
}
function deactivate() { }
//# sourceMappingURL=extension.js.map