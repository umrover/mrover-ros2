const { test, expect } = require('@playwright/test');

test.beforeEach(async ({ page }) => {
  await page.setViewportSize({ width: 1920, height: 1080 });
});

test('funnel controls visible on ScienceTask', async ({ page }) => {
  await page.goto('http://localhost:8080/ScienceTask');
  await page.waitForLoadState('networkidle');
  const controls = page.getByTestId('pw-funnel-controls');
  await expect(controls).toBeVisible({ timeout: 15000 });
});

test('6 site buttons visible', async ({ page }) => {
  await page.goto('http://localhost:8080/ScienceTask');
  await page.waitForLoadState('networkidle');
  await expect(page.getByTestId('pw-funnel-controls')).toBeVisible({ timeout: 15000 });

  for (let i = 0; i <= 5; i++) {
    await expect(page.getByTestId(`pw-funnel-site-${i}`)).toBeVisible();
  }
});

test('clicking site shows active state', async ({ page }) => {
  await page.goto('http://localhost:8080/ScienceTask');
  await page.waitForLoadState('networkidle');

  const siteBtn = page.getByTestId('pw-funnel-site-1');
  await expect(siteBtn).toBeVisible({ timeout: 15000 });
  await siteBtn.click();
  await expect(siteBtn).toHaveClass(/active/, { timeout: 5000 });
});
