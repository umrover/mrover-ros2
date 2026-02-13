const { test, expect } = require('@playwright/test');

test('sensor data visible on ScienceTask', async ({ page }) => {
  await page.goto('/ScienceTask');
  await expect(page.getByTestId('pw-sensor-view-all')).toBeVisible();
});

test('view all opens modal', async ({ page }) => {
  await page.goto('/ScienceTask');
  const viewAllBtn = page.getByTestId('pw-sensor-view-all');
  await expect(viewAllBtn).toBeVisible();
  await viewAllBtn.click();
  const modal = page.locator('.modal-backdrop');
  await expect(modal).toBeVisible();
});

test('csv button visible', async ({ page }) => {
  await page.goto('/ScienceTask');
  await expect(page.getByTestId('pw-sensor-csv-btn')).toBeVisible();
});
