const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('panorama button visible on ScienceTask', async ({ page }) => {
  await page.goto('/ScienceTask');
  await page.waitForLoadState('networkidle');
  const toggleOn = page.getByTestId('pw-panorama-start');
  await expect(toggleOn).toBeVisible({ timeout: 15000 });

  const toggleOff = page.getByTestId('pw-panorama-stop');
  await expect(toggleOff).toBeVisible({ timeout: 15000 });
});
