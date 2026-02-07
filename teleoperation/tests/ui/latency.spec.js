const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('benchmark page loads', async ({ page }) => {
  await page.goto('http://localhost:8080/dev');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-latency-start')).toBeVisible({ timeout: 15000 });
});

test('start button visible', async ({ page }) => {
  await page.goto('http://localhost:8080/dev');
  await page.waitForLoadState('networkidle');
  const startBtn = page.getByTestId('pw-latency-start');
  await expect(startBtn).toBeVisible({ timeout: 15000 });
  await expect(startBtn).toContainText('Start');
});

test('reset button visible', async ({ page }) => {
  await page.goto('http://localhost:8080/dev');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-latency-reset')).toBeVisible({ timeout: 15000 });
});

test('frequency input defaults to 10', async ({ page }) => {
  await page.goto('http://localhost:8080/dev');
  await page.waitForLoadState('networkidle');
  const freqInput = page.getByTestId('pw-latency-freq');
  await expect(freqInput).toBeVisible({ timeout: 15000 });
  await expect(freqInput).toHaveValue('10');
});
