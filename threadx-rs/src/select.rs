// Implementation of select basically the same as in embassy
use core::pin::Pin;

use either::Either;

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Select<A, B> {
    a: A,
    b: B,
}

pub fn select<A, B>(a: A, b: B) -> Select<A, B>
where
    A: Future,
    B: Future,
{
    Select { a, b }
}

impl<A: Unpin, B: Unpin> Unpin for Select<A, B> {}

impl<A, B> Future for Select<A, B>
where
    A: Future,
    B: Future,
{
    type Output = Either<A::Output, B::Output>;

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let this = unsafe { self.get_unchecked_mut() };
        let pin_fut_a = unsafe { Pin::new_unchecked(&mut this.a) };
        let pin_fut_b = unsafe { Pin::new_unchecked(&mut this.b) };

        if let core::task::Poll::Ready(val) = pin_fut_a.poll(cx) {
            return core::task::Poll::Ready(Either::Left(val));
        }

        if let core::task::Poll::Ready(val) = pin_fut_b.poll(cx) {
            return core::task::Poll::Ready(Either::Right(val));
        }

        core::task::Poll::Pending
    }
}
