const { test, expect } = require('@playwright/test');

test('panorama button visible on ScienceTask', async ({ page }) => {
  await page.goto('/ScienceTask');
  const toggle = page.getByTestId('pw-panorama-toggle');
  await expect(toggle).toBeVisible();
});

test('button shows Start initially', async ({ page }) => {
  await page.goto('/ScienceTask');
  const toggle = page.getByTestId('pw-panorama-toggle');
  await expect(toggle).toBeVisible();
  await expect(toggle).toContainText('Start');
});
