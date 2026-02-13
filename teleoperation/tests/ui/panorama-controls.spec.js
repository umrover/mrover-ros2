const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('panorama button visible on ScienceTask', async ({ page }) => {
  await page.goto('http://localhost:8080/ScienceTask');
  await page.waitForLoadState('networkidle');
  const toggle = page.getByTestId('pw-panorama-toggle');
  await expect(toggle).toBeVisible({ timeout: 15000 });
});

test('button shows Start initially', async ({ page }) => {
  await page.goto('http://localhost:8080/ScienceTask');
  await page.waitForLoadState('networkidle');
  const toggle = page.getByTestId('pw-panorama-toggle');
  await expect(toggle).toBeVisible({ timeout: 15000 });
  await expect(toggle).toContainText('Start');
});
