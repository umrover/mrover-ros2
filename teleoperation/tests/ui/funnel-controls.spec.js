const { test, expect } = require('@playwright/test');

test('funnel controls visible on ScienceTask', async ({ page }) => {
  await page.goto('/ScienceTask');
  const controls = page.getByTestId('pw-funnel-controls');
  await expect(controls).toBeVisible();
});

test('6 site buttons visible', async ({ page }) => {
  await page.goto('/ScienceTask');
  await expect(page.getByTestId('pw-funnel-controls')).toBeVisible();

  for (let i = 0; i <= 5; i++) {
    await expect(page.getByTestId(`pw-funnel-site-${i}`)).toBeVisible();
  }
});

test('clicking site shows active state', async ({ page }) => {
  await page.goto('/ScienceTask');

  const siteBtn = page.getByTestId('pw-funnel-site-1');
  await expect(siteBtn).toBeVisible();
  await siteBtn.click();
  await expect(siteBtn).toHaveClass(/active/);
});
