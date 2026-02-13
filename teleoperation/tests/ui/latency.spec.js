const { test, expect } = require('@playwright/test');

test('benchmark page loads', async ({ page }) => {
  await page.goto('/dev');
  await expect(page.getByTestId('pw-latency-start')).toBeVisible();
});

test('start button visible', async ({ page }) => {
  await page.goto('/dev');
  const startBtn = page.getByTestId('pw-latency-start');
  await expect(startBtn).toBeVisible();
  await expect(startBtn).toContainText('Start');
});

test('reset button visible', async ({ page }) => {
  await page.goto('/dev');
  await expect(page.getByTestId('pw-latency-reset')).toBeVisible();
});

test('frequency input defaults to 10', async ({ page }) => {
  await page.goto('/dev');
  const freqInput = page.getByTestId('pw-latency-freq');
  await expect(freqInput).toBeVisible();
  await expect(freqInput).toHaveValue('10');
});
