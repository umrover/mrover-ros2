import * as vscode from 'vscode';
import { promises as fs } from 'fs';

async function callInInteractiveTerminal(command: string): Promise<vscode.TerminalExitStatus> {
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
                } else {
                    reject(new Error('Terminal exited with undefined status'));
                }
            }
        });
    });
}

async function readExecutables(directoryPath: string): Promise<string[]> {
    try {
        const entries = await fs.readdir(directoryPath);
        return entries.filter(f => !f.endsWith('.py') && !f.endsWith('.sh'));
    } catch (err) {
        vscode.window.showErrorMessage(`Failed to read directory: ${err}`);
        return [];
    }
}

export function activate(context: vscode.ExtensionContext): void {
    const disposable = vscode.commands.registerCommand(
        'file-selector.fileSelect',
        async (directory: string): Promise<string | undefined> => {
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
        }
    );
    context.subscriptions.push(disposable);
}

export function deactivate(): void {}
