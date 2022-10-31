jQuery(function($) {
	// Home Slider
        var $owlHome = $('.home-slider');
        var $owlHomeThumb = $(".home-slider-thumbs");
        var $slidesPerPage = 5;
        var $owledSecondary = true;
        $owlHome.owlCarousel({
            rtl: $("html").attr("dir") == 'rtl' ? true : false,
            items: 1,
            autoplay: true,
            autoplayTimeout: 10000,
            margin: 0,
            loop: true,
            dots: false,
            nav: true,
            navText: [slider_settings.arrowLeft, slider_settings.arrowRight],
            singleItem: true,
            transitionStyle: "fade",
            touchDrag: true,
            mouseDrag: true,
            slideSpeed: 2000,
            responsiveRefreshRate: 200,
            responsive: {
                0: {
                    nav: false
                },
                992: {
                    nav: true
                }
            }
        }).on('changed.owl.carousel', owlPosition);
        $owlHomeThumb.on('initialized.owl.carousel', function() {
            $owlHomeThumb.find(".owl-item").eq(0).addClass("current");
        }).owlCarousel({
            items: $slidesPerPage,
            dots: false,
            nav: false,
            margin: 20,
            smartSpeed: 200,
            slideSpeed: 500,
            touchDrag: true,
            mouseDrag: true,
            slideBy: $slidesPerPage,
            responsiveRefreshRate: 100
        }).on('changed.owl.carousel', owlPosition2);
        function owlPosition(el) {
            var count = el.item.count - 1;
            var current = Math.round(el.item.index - (el.item.count / 2) - .5);
            if (current < 0) {
                current = count;
            }
            if (current > count) {
                current = 0;
            }
            $owlHomeThumb.find(".owl-item").removeClass("current").eq(current).addClass("current");
            var onscreen = $owlHomeThumb.find('.owl-item.active').length - 1;
            var start = $owlHomeThumb.find('.owl-item.active').first().index();
            var end = $owlHomeThumb.find('.owl-item.active').last().index();
            if (current > end) {
                $owlHomeThumb.data('owl.carousel').to(current, 100, true);
            }
            if (current < start) {
                $owlHomeThumb.data('owl.carousel').to(current - onscreen, 100, true);
            }
        }
        function owlPosition2(el) {
            if ($owledSecondary) {
                var number = el.item.index;
                $owlHome.data('owl.carousel').to(number, 100, true);
            }
        }
        $owlHomeThumb.on("click", ".owl-item", function(e) {
            e.preventDefault();
            var number = $(this).index();
            $owlHome.data('owl.carousel').to(number, 300, true);
        });
        $owlHome.owlCarousel();
        $owlHome.on('translate.owl.carousel', function (event) {
            var data_anim = $("[data-animation]");
            data_anim.each(function() {
                var anim_name = $(this).data('animation');
                $(this).removeClass('animated ' + anim_name).css('opacity', '0');
            });
        });
        $("[data-delay]").each(function() {
            var anim_del = $(this).data('delay');
            $(this).css('animation-delay', anim_del);
        });
        $("[data-duration]").each(function() {
            var anim_dur = $(this).data('duration');
            $(this).css('animation-duration', anim_dur);
        });
        $owlHome.on('translated.owl.carousel', function() {
            var data_anim = $owlHome.find('.owl-item.active').find("[data-animation]");
            data_anim.each(function() {
                var anim_name = $(this).data('animation');
                $(this).addClass('animated ' + anim_name).css('opacity', '1');
            });
        });
});