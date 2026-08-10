const { defineConfig } = require('@playwright/test');

module.exports = defineConfig({
  testDir: './tests',
  timeout: 60000,
  expect: {
    timeout: 10000,
  },
  use: {
    baseURL: 'http://localhost:8080',
    viewport: { width: 1920, height: 1080 },
    screenshot: 'only-on-failure',
    video: 'retain-on-failure',
    trace: 'on',
  },
  reporter: [
    ['list'],
    ['html', { open: 'never' }],
  ],
  webServer: [
        {
            name: 'Backend',
            command: 'python3 server.py',
            cwd: '.',
            url: 'http://localhost:8000',
            timeout: 60000,
            reuseExistingServer: !process.env.CI,
        },
        {
            name: 'Frontend',
            command: 'bun --bun run dev',
            cwd: './frontend',
            url: 'http://localhost:8080',
            timeout: 60000,
            reuseExistingServer: !process.env.CI,
        },
    ],
});
